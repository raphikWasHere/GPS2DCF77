/* 
 * Transmisor DCF77 con patrón GPS
 * No utiliza librerías para GPS ni PWM
 * * :P 2022 raphik
 * 
 * El sistema mantiene la hora UTC
 * Se visualiza la hora CET/CEST (se calcula automáticamente)
 * La transmisión DCF77 es del minuto siguiente a la hora CES/CEST
 DCF77
 Position 0         5        10        15        20        25        30        35        40        45        50        55      59
          |         |         |         |         |         |         |         |         |         |         |         |       |
 VALUE    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,1,0,0,0,1,0,0,1,0,0,1,1,0,1,1,0,1,0,1,0,1,0,1,0,0,0,1,0,0,1,0,0,0,1,0,0,0,0
          X |___________________________________| X |___________| P |_________| P |_________| |___| |_______| |_____________| P
 CONTENT     DATOS METEOROLÓGICOS                    MINUTOS         HORAS         D.MES      D.SEM  MES       AÑO
*/

#include <TimeLib.h>
#include <SoftwareSerial.h>

#define DCF_freq 77500        // frecuencia de la portadora
#define ANTENNA 3             // antena
#define MODULATION 0
#define NO_MODULATION 127
#define S_RX    9             // pin RX (GPS-TX) 
#define S_TX    8             // pin TX (GPS-RX) no es necesario conectar
// #define OLD_GPS 1024*7*86400  // 1024 semanas expresadas en segundos
// #define CET     3600          // horario de invierno (UTC+1)
// #define CEST    7200          // horario de venaro (UTC+2) 

SoftwareSerial SerialGPS(S_RX, S_TX);

int DCF77[60];                // tabla con los 60 bits DCF77
int actSecond;
unsigned long prevMillis, actMillis;

// código BDC
int BCD[10][4] = { {0,0,0,0},{0,0,0,1},{0,0,1,0},{0,0,1,1},{0,1,0,0},{0,1,0,1},{0,1,1,0},{0,1,1,1},{1,0,0,0},{1,0,0,1} };


void setup(void) {
  // configuración del timer 2 a 77699 Hz
  pinMode (ANTENNA, OUTPUT) ;
  TCCR2A = 0x23 ;
  TCCR2B = 0x09 ; // mode 7, clock prescale by 1
  OCR2A = 16000000 / 77500 - 1 ; // = 206 tics de reloj => frecuencia real 16000000 / 206 = 77669 Hz
  OCR2B = 16000000 / 77500 / 2 ; // = 103 tics de reloj (50% duty cycle)
 
  Serial.begin(115200);
  SerialGPS.begin(38400);  // initialize software serial at 38400 baud
  Serial.println("GPS CLOCK (UTC+1)");
  prevMillis = millis();
}

void loop()
{
  while( SerialGPS.available() ) {
    String nmeaSentence = SerialGPS.readStringUntil('\n');
    if( timeStatus() != timeSet && nmeaSentence.startsWith("$GPRMC") ) {
      // función que devuelve el tiempo Unix que marca el GPS a partir de la sentencia nmea
      time_t timestampGPS = nmea2UnixTime( nmeaSentence );
      // sincronizar el sistema con la hora del GPS (hora UTC)
      setTime( timestampGPS );
      if( timeStatus() == timeSet ) {
        fillDCF77();
      }
    }
  }

  if( timeStatus() == timeSet ) {
    actSecond = second();
    actMillis = millis();
    if( actMillis >= prevMillis + 1000UL ) {
      prevMillis = actMillis;
      if( actSecond < 59 ) {
        analogWrite(ANTENNA,MODULATION); DCF77[actSecond] ? delay(200) : delay(100); analogWrite(ANTENNA,NO_MODULATION);
      } else {
        fillDCF77();
      }
      showLocalTime();
    }
  }  
}


time_t nmea2UnixTime( String nmeaSentence ){
  String strTime, strDate;
  // despreciar el checksum; por ahora no se comprueba. Igual lo añado más adelante.
  String nmeaData = nmeaSentence.substring(1, nmeaSentence.length() - 4);
  byte iStart = nmeaData.indexOf( ',', 0);
  byte iStop;
  for ( byte i = 0; i < 9 ; i++ ) {
    iStop = nmeaData.indexOf( ',', iStart + 1 );
    // averiguar la hora, despreciando los milisegundos
    if ( i == 0 ) strTime = nmeaData.substring( iStart + 1, iStop - 4 );
    // averiguar la fecha
    if ( i == 8 ) strDate = nmeaData.substring( iStart + 1, iStop );
    iStart = iStop;
  }
  // calcula el tiempo Unix señalado por el GPS
  time_t unixTime = unixEpochTime( strTime.substring(0,2).toInt(), strTime.substring(2,4).toInt(), strTime.substring(4,6).toInt(),
                                   strDate.substring(0,2).toInt(), strDate.substring(2,4).toInt(), strDate.substring(4,6).toInt() );
  // si se detecta que el GPS es viejuno, corregir agregando 1024 semanas más (expresadas en segundos)
  if( unixTime < 1554508800) unixTime += 1024 * 7 * 86400;
  return unixTime;
}

time_t unixEpochTime( int horo, int minuto, int sekundo, int tago, int monato, int jaro ) {
  tmElements_t x;
  x.Hour = horo; x.Minute = minuto; x.Second = sekundo; x.Day = tago; x.Month = monato;
  // si jaro es menor que 100 (2 cifras), se le suma 2000
  if( jaro < 100 ) jaro += 2000;
  x.Year = jaro - 1970;
  return makeTime(x);
}

boolean isDST( time_t timestamp ) {
  // función que devuelve true si timestamp están dentro del horario de verano
  time_t oneOclockTimestamp; byte lastWeekdayOfMonth;

  oneOclockTimestamp = unixEpochTime( 1, 0, 0, 31, 3, year(timestamp) );     // 31 de marzo, una de la mañana
  lastWeekdayOfMonth = weekday( oneOclockTimestamp ) - 1;
  time_t startDST = oneOclockTimestamp - lastWeekdayOfMonth * 86400;  // inicio del horario de verano

  oneOclockTimestamp = unixEpochTime( 1, 0, 0, 31, 10, year(timestamp) );    // 31 de octubre, una de la mañana
  lastWeekdayOfMonth = weekday( oneOclockTimestamp ) - 1;
  time_t stopDST = oneOclockTimestamp - lastWeekdayOfMonth * 86400;   // fin del horario de verano

  //Serial.print("Start DST: "); Serial.println( startDST );
  //Serial.print("Stop DST: "); Serial.println( stopDST);

  return ( timestamp >= startDST && timestamp < stopDST ) ? true : false;
}

void showLocalTime() {
  time_t t = now();
  time_t localTime = isDST(t) ? (t+7200) : (t+3600);
  Serial.print(day(localTime)); Serial.print("/"); Serial.print(month(localTime)); Serial.print("/"); Serial.print(year(localTime)); Serial.print("  ");
  Serial.print(hour(localTime)); Serial.print(":"); Serial.print(minute(localTime)); Serial.print(":"); Serial.print(second(localTime));
  if(isDST(t)) Serial.println(" CEST"); else Serial.println(" CET");

  /*
  Serial.println("0         5        10        15        20        25        30        35        40        45        50        55      59");
  Serial.println("|         |         |         |         |         |         |         |         |         |         |         |       |");
  for(int i = 0; i < 59; i++) {
    Serial.print(DCF77[i]);
    Serial.print(",");
  }
  Serial.println("0");
  Serial.println("X |___________________________________| X |___________| P |_________| P |_________| |___| |_______| |_____________| P");
  Serial.println("   DATOS METEOROLÓGICOS                    MINUTOS         HORAS         D.MES      D.SEM  MES       AÑO");
  */
}

void fillDCF77() {
  int msb, lsb, n, aux;
  // se transmite el minuto siguiente al actual
  time_t dcfTime = now();
  int DST = isDST(dcfTime) ? 1 : 0;
  dcfTime = DST ? dcfTime+7200+120 : dcfTime+3600+120;

  // poner los 20 primeros bits a 0
  for (n = 0; n < 20; n++) DCF77[n] = 0;
  
  // seleccionar hora CET / CEST
  if( DST ) DCF77[17] = 1; else DCF77[18] = 1;
  
  // el bit 20 siempre a 1
  DCF77[20] = 1;

  // cálculo de los bits minutos
  aux = minute(dcfTime);
  lsb = aux%10; msb = aux/10;
  DCF77[21] = BCD[lsb][3]; DCF77[22] = BCD[lsb][2]; DCF77[23] = BCD[lsb][1]; DCF77[24] = BCD[lsb][0];
  DCF77[25] = BCD[msb][3]; DCF77[26] = BCD[msb][2]; DCF77[27] = BCD[msb][1];
  // cálculo de la paridad de los minutos
  aux = 0; for (n = 21; n < 28; n++) aux += DCF77[n]; DCF77[28] = aux%2;

  // cálculo de los bits horas
  aux = hour(dcfTime);
  lsb = aux%10; msb = aux/10;
  DCF77[29] = BCD[lsb][3]; DCF77[30] = BCD[lsb][2]; DCF77[31] = BCD[lsb][1]; DCF77[32] = BCD[lsb][0];
  DCF77[33] = BCD[msb][3]; DCF77[34] = BCD[msb][2];
  // cálculo de la paridad de las horas
  aux = 0; for (n = 29; n < 35; n++) aux += DCF77[n]; DCF77[35] = aux%2;
  
  // cálculo de los bits del día del mes
  aux = day(dcfTime);
  lsb = aux%10; msb = aux/10;
  DCF77[36] = BCD[lsb][3]; DCF77[37] = BCD[lsb][2]; DCF77[38] = BCD[lsb][1]; DCF77[39] = BCD[lsb][0];
  DCF77[40] = BCD[msb][3]; DCF77[41] = BCD[msb][2];

  // cálculo de los bits del día de la semana
  aux = weekday(dcfTime);
  lsb = aux%10 - 1; if( lsb == 0 ) lsb = 7;
  DCF77[42] = BCD[lsb][3]; DCF77[43] = BCD[lsb][2]; DCF77[44] = BCD[lsb][1];

  // cálculo de los bits del mes
  aux = month(dcfTime);
  lsb = aux%10; msb = aux/10;
  DCF77[45] = BCD[lsb][3]; DCF77[46] = BCD[lsb][2]; DCF77[47] = BCD[lsb][1]; DCF77[48] = BCD[lsb][0];
  DCF77[49] = BCD[msb][3];
  
  // cálculo de los bits del año
  aux = year(dcfTime);
  lsb = aux%100%10; msb = aux%100/10;
  DCF77[50] = BCD[lsb][3]; DCF77[51] = BCD[lsb][2]; DCF77[52] = BCD[lsb][1]; DCF77[53] = BCD[lsb][0];
  DCF77[54] = BCD[msb][3]; DCF77[55] = BCD[msb][2]; DCF77[56] = BCD[msb][1]; DCF77[57] = BCD[msb][0];
  // cálculo de la paridad de la fecha
  aux = 0; for (n = 36; n < 58; n++) aux += DCF77[n]; DCF77[58] = aux%2;

  // último pulso
  DCF77[59] = -1; // No hay modulación
  
}
