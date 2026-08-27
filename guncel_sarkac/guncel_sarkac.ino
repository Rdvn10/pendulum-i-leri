// ==========================================
// ROTARY INVERTED PENDULUM 
// ==========================================
#include <math.h>

// --- TANIMLAMALAR ---
 const int sarkac_A = 2; 
 const int sarkac_B = 4; 
 const int motor_A = 3; 
 const int motor_B = 5; 

 const int RPWM = 10;
 const int LPWM = 9;

 const double m = 0.01;
 const double l = 0.12;
 const double eylemsizlik_moment = m*l*l/3.0;
 const double g = 9.81;

 const double hedef_enerji = 2*m*g*l;
 double mevcut_enerji = 0;

// --- DURUMLAR ---
 enum SistemDurumu {
  SWING_UP,    // Yatayın altında enerji pompalama
  YAKALAMA,    // Yatayın üstünde süzülerek yavaşlama
  DENGELEME,   // Tepe noktasında tamamen sessiz duruş (Motor = 0)
  TAMPON_BOLGE // Mikro titreşim ve aktif sönümleme (Aktif Fren)
 };
 SistemDurumu sistem_durumu = SWING_UP;

// --- KASKAD KONTROL KAZANÇLARI ---
 double Kp_dis = -0.09;   // Dış döngü oransal kazanç (tune edin)
 double Kd_dis = -0.07;   // Dış döngü türev kazanç (tune edin)

 double Kp_ic = 2.5;      // İç döngü oransal kazanç V/rad (tune edin)
 double Kd_ic = 0.2;      // İç döngü türev kazanç V/(rad/s) (tune edin)

 double Ke = 95.0;

// --- DURUM DEĞİŞKENLERİ ---
 double sarkac_konum = 0.0;
 double motor_konum = 0.0;
 double sarkac_hiz = 0.0;
 double motor_hiz = 0.0;
 double anlik_sarkac = 0.0;
 const int min_pwm = 10;

 double sarkac_oncekikonum_rad = 0.0;
 double motor_oncekikonum_rad = 0.0;

 const double SARKAC_PALS_TO_RAD = (2.0 * PI) / 1200.0;
 const double MOTOR_PALS_TO_RAD  = (2.0 * PI) / 200.0;
 const double hedef_pals = 600.0; // Eski 300 artık 600

 unsigned long onceki_zaman_mikro = 0;
 volatile long sarkac_pals_sayaci = 0;
 volatile long motor_pals_sayaci = 0;

// --- KESMELER ---
 void sarkac_ISR() {
  if (digitalRead(sarkac_A) == digitalRead(sarkac_B)) sarkac_pals_sayaci++; 
  else sarkac_pals_sayaci--;
 }

 void motor_ISR() {
  if (digitalRead(motor_A) == digitalRead(motor_B)) motor_pals_sayaci++; 
  else motor_pals_sayaci--;
 }

// --- FONKSİYONLAR ---
 double hiz_hesapla(double mevcut, double &eski, double dt) {
  double hiz = (mevcut - eski) / dt;
  eski = mevcut;
  return hiz;
 }

 int sign(double deger) {
  return constrain(deger*10,-1,1);
 }

 void motorgucver(double V_uygulanacak, int pwm) {
  if (V_uygulanacak > 0) {
    analogWrite(RPWM, pwm);
    digitalWrite(LPWM, LOW);
  }
  else if (V_uygulanacak < 0) {
    digitalWrite(RPWM, LOW);
    analogWrite(LPWM, pwm);
  }
  else {
    digitalWrite(RPWM, LOW);
    digitalWrite(LPWM, LOW);
  }
 }

// --- SETUP ---
 void setup() {
  Serial.begin(115200); 
  
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(sarkac_A, INPUT_PULLUP);
  pinMode(sarkac_B, INPUT_PULLUP);
  pinMode(motor_A, INPUT_PULLUP);
  pinMode(motor_B, INPUT_PULLUP);

  // RISING yerine CHANGE yapıldı (2X Çözünürlük aktif)
  attachInterrupt(digitalPinToInterrupt(sarkac_A), sarkac_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor_A), motor_ISR, CHANGE);

  onceki_zaman_mikro = micros();
 }

// --- LOOP ---
 void loop() {
  unsigned long suan_mikro = micros();
  double dt = (suan_mikro - onceki_zaman_mikro) / 1000000.0; 

  if (dt >= 0.01) { // 100 Hz Döngü
    onceki_zaman_mikro = suan_mikro;

    long anlik_sarkac_pals, anlik_motor_pals;
    noInterrupts(); 
    anlik_sarkac_pals = sarkac_pals_sayaci;
    anlik_motor_pals = motor_pals_sayaci;
    interrupts();

    // --- HAM DEĞERLER ---
    anlik_sarkac = (double)anlik_sarkac_pals;
    double motor_hamkonum = (double)anlik_motor_pals;

    // --- HIZ HESABI (HAM değerlerden — filtre/modulo YOK, faz gecikmesi yok) ---
    double sarkac_ham_rad = anlik_sarkac * SARKAC_PALS_TO_RAD;
    double sarkacfiltrelenmemis_hiz = hiz_hesapla(sarkac_ham_rad, sarkac_oncekikonum_rad, dt);
    sarkac_hiz = sarkacfiltrelenmemis_hiz * 0.4 + sarkac_hiz * 0.6;

    double motor_ham_rad = motor_hamkonum * MOTOR_PALS_TO_RAD;
    double motor_filtrelenmemishiz = hiz_hesapla(motor_ham_rad, motor_oncekikonum_rad, dt);
    motor_hiz = motor_filtrelenmemishiz * 0.4 + motor_hiz * 0.6;

    // --- SARKAÇ KONUM FİLTRESİ (Dairesel EMA — modulo uyumlu) ---
    double sarkac_mod = fmod(anlik_sarkac, 1200.0);
    if (sarkac_mod < 0) sarkac_mod += 1200.0;

    double sarkac_fark = sarkac_mod - sarkac_konum;
    if (sarkac_fark > 600.0) sarkac_fark -= 1200.0;   // Kısa yoldan fark al
    if (sarkac_fark < -600.0) sarkac_fark += 1200.0;
    sarkac_konum = sarkac_konum + 0.8 * sarkac_fark;  // %80 yeni, %20 eski
    sarkac_konum = fmod(sarkac_konum, 1200.0);
    if (sarkac_konum < 0) sarkac_konum += 1200.0;

    // --- MOTOR KONUM FİLTRESİ (Standart EMA) ---
    motor_konum = motor_hamkonum * 0.8 + motor_konum * 0.2;

    // --- RADYAN DÖNÜŞÜMÜ (Filtrelenmiş konumlardan) ---
    double sarkac_konumrad = sarkac_konum * SARKAC_PALS_TO_RAD;
    double motor_konumrad = motor_konum * MOTOR_PALS_TO_RAD;

    double V_uygulanacak = 0.0;
    int pwm_uygulanacak = 0;

    // Sarkaç tepeye uzaklığı (Mutlak değer)
    double tepe_uzakligi = fabs(sarkac_konum - hedef_pals);

    // --- DURUM MAKİNESİ (Sınırlar 2 katına çıkarıldı) ---
switch (sistem_durumu) {

 case SWING_UP:
        // GEÇİŞ: Sarkaç 90 dereceyi (300 pals) geçtiyse YAKALAMA moduna geç
        if (tepe_uzakligi <= 300.0) {
          sistem_durumu = YAKALAMA; // BURASI DÜZELTİLDİ: Sistem_durumu -> sistem_durumu
          break;
        }

        {
          double itki_yonu;
          // İlk Hareketi Başlat (Kickstart)
          if (fabs(sarkac_hiz) < 0.05) {
            itki_yonu = 0.0; 
          } else {
            itki_yonu = (sarkac_hiz > 0) ? 1.0 : -1.0;
          }

          // Cosinus çarpanını kaldırdık ki sarkaç yükselirken motor gücü kesilmesin!
          double merkezleme_torku = -2.0 * motor_konumrad - 0.15 * motor_hiz;
          
          // KAZANCI 4.0 YAPTIK (Açı çarpanı yok, saf itki)
          // NOT: Sarkaç her salınımda genliğini büyütmüyor, frenliyorsa buradaki -3.0'ı +3.0 veya tersi yapın.
          V_uygulanacak = (-3.0 * itki_yonu) + merkezleme_torku;

          if (motor_hamkonum >= 75.0 && V_uygulanacak > 0) V_uygulanacak = 0; 
          else if (motor_hamkonum <= -75.0 && V_uygulanacak < 0) V_uygulanacak = 0; 

          int pwm_ham = fabs(V_uygulanacak) * (255.0 / 12.0);
          
          if (pwm_ham > 0) {
            // SINIRI 150 YAPTIK: Motora artık %60'a kadar güç kullanma izni verdik!
            pwm_uygulanacak = constrain(pwm_ham + min_pwm, 0, 80); 
            pwm_uygulanacak =0;
          } else {
            pwm_uygulanacak = 0;
          }
        }
        break;

 case YAKALAMA:
        // GEÇİŞ 1: Sarkaç Tampon Bölgeye (90 pals) girdiyse oraya geç
        if (tepe_uzakligi <= 90.0) { 
          sistem_durumu = TAMPON_BOLGE;
          break;
        }
        // GEÇİŞ 2: Sarkaç 90°'nin (300 pals) altına düştüyse SWING_UP'a dön
        else if (tepe_uzakligi > 300.0) {
          sistem_durumu = SWING_UP;
          break;
        }

        
        {
          double sarkac_konumhata = (sarkac_konum - hedef_pals) * SARKAC_PALS_TO_RAD;
          double K_ideal_hiz = 2.0; 
          double hedef_sarkac_hizi = -1.0 * K_ideal_hiz * sarkac_konumhata; 

          if (fabs(sarkac_hiz) <= fabs(hedef_sarkac_hizi)) {
            V_uygulanacak = 0.0;
            pwm_uygulanacak = 0;
          } 
          else {
            
            pwm_uygulanacak = 0; // test durdu
          }
        }
        break;

 case DENGELEME:
        // Sarkaç çok yakın : // Taş gibi dursun
        if (tepe_uzakligi > 14.0) {
          sistem_durumu = TAMPON_BOLGE; 
          break;
        }

        V_uygulanacak = 0.0;
        pwm_uygulanacak = 0;
        break;

 case TAMPON_BOLGE:
        // GEÇİŞ: Sarkaç tepe noktasından çok uzaklaştıysa YAKALAMA veya SWING_UP'a dön
        if (tepe_uzakligi > 90.0) { // Sınırı sisteminize göre ayarlayın
          sistem_durumu = YAKALAMA; 
          break;
        }

        // ==========================================
        // KASKAD KONTROL (CASCADE PD CONTROL)
        // ==========================================

        // --- 1. DIŞ DÖNGÜ (MOTOR KONUM KONTROLÜ) --- [RADYAN]
        {
          double motor_hata_rad = 0.0 - motor_konumrad; // Hedef = 0 (merkez)

          // Motor hatası → sarkaç hedef sapması (radyan cinsinde)
          // Not: Yön yanlışsa Kp_dis işaretini tersleyin
          double hedef_sarkac_sapmasi_rad = Kp_dis * motor_hata_rad - Kd_dis * motor_hiz; 
          // GÜVENLİK: Maks ±8 derece (0.14 rad) sarkaç sapması
          hedef_sarkac_sapmasi_rad = constrain(hedef_sarkac_sapmasi_rad, -0.14, 0.14);

          // --- 2. İÇ DÖNGÜ (SARKAÇ DENGE KONTROLÜ) --- [RADYAN]
          double hedef_sarkac_rad = (hedef_pals * SARKAC_PALS_TO_RAD) + hedef_sarkac_sapmasi_rad;
          double sarkac_hata_rad = hedef_sarkac_rad - sarkac_konumrad;

          // PD Kontrolcü (tutarlı birimler: rad ve rad/s)
          V_uygulanacak = (Kp_ic * sarkac_hata_rad) - (Kd_ic * sarkac_hiz);

          // --- GÜVENLİK VE ÇIKIŞ ---
          // Donanımsal sınır koruması
          if (motor_hamkonum >= 75 && V_uygulanacak > 0) V_uygulanacak = 0; 
          else if (motor_hamkonum <= -75 && V_uygulanacak < 0) V_uygulanacak = 0; 

          // Voltajı PWM'e çevir
          int pwm_ham = fabs(V_uygulanacak) * (255.0 / 12.0); // 12V besleme varsayımı
          
          // Ölü Bant (Deadband) Telafisi: Motorun sürtünmeyi yenip harekete geçtiği minimum PWM
          if (pwm_ham > 0) { 
            // Hesaplanan saf gücün üzerine HER ZAMAN sürtünme eşiğini ekle
            pwm_uygulanacak = constrain(pwm_ham + min_pwm, 0, 50); 
          } else {
            pwm_uygulanacak = 0; // Hata yoksa motor dursun
          }
        }
        break;
    }
    
    // printler
    motorgucver(V_uygulanacak, pwm_uygulanacak);

    double rad2deg = 180.0 / PI;

   
    Serial.print("Mod:");
    if (sistem_durumu == DENGELEME) Serial.print("DENGE");
    else if (sistem_durumu == YAKALAMA) Serial.print("YAKAL");
    else if (sistem_durumu == TAMPON_BOLGE) Serial.print("TAMPN");
    else Serial.print("SWING");
    
    Serial.print("\t M_Konum:"); 
    Serial.print(motor_konumrad * rad2deg, 2);
    Serial.print("\t S_Konum:"); 
    Serial.print(sarkac_konumrad * rad2deg, 2);
    Serial.print("\t M_Hiz:"); 
    Serial.print(motor_hiz * rad2deg, 2);
    Serial.print("\t S_Hiz:"); 
    Serial.print(sarkac_hiz * rad2deg, 2); 
    Serial.print("\t PWM:"); 
    Serial.println(pwm_uygulanacak); 
  }
 }