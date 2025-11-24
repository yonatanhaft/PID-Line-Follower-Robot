#include <QTRSensors.h>

// הגדרת פינים למנועים (PWM ודיגיטליים)
#define PWMA 3   // פין PWM למנוע שמאלי - משמש לשליטה במהירות המנוע השמאלי
#define AIN1 5   // פין כיוון למנוע שמאלי - אחד משני פיני השליטה על כיוון המנוע השמאלי
#define AIN2 6   // פין כיוון למנוע שמאלי - השני משני פיני השליטה על כיוון המנוע השמאלי
#define PWMB 9   // פין PWM למנוע ימני - משמש לשליטה במהירות המנוע הימני
#define BIN1 10  // פין כיוון למנוע ימני - אחד משני פיני השליטה על כיוון המנוע הימני
#define BIN2 11  // פין כיוון למנוע ימני - השני משני פיני השליטה על כיוון המנוע הימני

// משתנים לשליטה במהירות
int MAX_SPEED = 145; // מהירות מקסימלית של המנועים (ערך בין 0 ל-255)
int BASE_SPEED = 105;  // מהירות בסיסית של הרובוט בנסיעה ישרה

QTRSensors qtr; // יצירת אובייקט לספריית חיישני QTR

const uint8_t SensorCount = 5; // מספר החיישנים המחוברים
uint16_t sensorValues[SensorCount]; // מערך שיאחסן את קריאות הערכים האנלוגיים מכל חיישן
int threshold[SensorCount]; // מערך שיאחסן את ערכי הסף (הממוצע בין שחור ללבן) עבור כל חיישן

// פרמטרים של בקרת PID (פרופורציונלי, אינטגרלי, נגזר)
float Kp = 1.65; // קבוע פרופורציונלי - משפיע על תגובת הרובוט לשגיאה הנוכחית
float Ki = 0.0;     // קבוע אינטגרלי - עוזר לצמצם שגיאות מצטברות לאורך זמן
float Kd = 3.70;     // קבוע נגזר - משפיע על תגובת הרובוט לשינוי בשגיאה

uint8_t multiP = 1, multiI = 1, multiD = 1; // משתנים המשמשים לחלוקת קבועי ה-PID בחזקות 10 (לכוונון עדין)
float Pvalue, Ivalue, Dvalue; // משתנים שיאחסנו את רכיבי ה-PID השונים

uint16_t position; // משתנה שיאחסן את המיקום המשוקלל של הקו השחור מתחת לחיישנים
int P, D, I, previousError, PIDvalue, error; // משתנים המשמשים בחישוב בקרת ה-PID
int lsp, rsp; // משתנים שיאחסנו את מהירויות המנועים השמאלי והימני לאחר תיקון ה-PID
uint16_t lastPosition = 0; // משתנה שיאחסן את המיקום האחרון של הקו השחור שזוהה
bool onLine = true; // משתנה בוליאני המציין האם לפחות אחד החיישנים רואה קו שחור

void setup() {
  // הגדרת הפינים של המנועים כפלט
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // הגדרת החיישנים
  qtr.setTypeAnalog(); // הגדרת סוג החיישנים כאנלוגיים
  qtr.setSensorPins((const uint8_t[]){A1, A2, A3, A4, A5}, SensorCount); // הגדרת פיני הארדואינו שאליהם מחוברים החיישנים

  pinMode(LED_BUILTIN, OUTPUT); // הגדרת פין הלד המובנה כפלט
  digitalWrite(LED_BUILTIN, HIGH); // הדלקת הלד המובנה בזמן הכיול

  // כיול החיישנים - קורא ערכים מהחיישנים כאשר הם רואים לבן ושחור כדי לקבוע את הטווחים
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // כיבוי הלד המובנה לאחר סיום הכיול

  // חישוב ערכי הסף עבור כל חיישן - הסף הוא נקודת האמצע בין הערך המינימלי למקסימלי שנקלט במהלך הכיול
  for (uint8_t i = 0; i < SensorCount; i++) {
    threshold[i] = (qtr.calibrationOn.minimum[i] + qtr.calibrationOn.maximum[i]) / 2;
  }

  delay(1000); // השהיה של 1 שניה לפני תחילת הלולאה הראשית
}

void loop() {
  robot_control(); // קריאה לפונקציה המרכזית השולטת ברובוט
}

void robot_control() {
  position = readLine(); // קריאה לפונקציה שקוראת את חיישני הקו ומחזירה את מיקום הקו
  error = 2000 - position; // חישוב השגיאה - המרכז של החיישנים (עבור 5 חיישנים) פחות מיקום הקו הנוכחי
  PID_Linefollow(error); // קריאה לפונקציה שמבצעת את בקרת ה-PID על סמך השגיאה
}

// פונקציה שמחזירה את המיקום האחרון של הקו השחור שזוהה
uint16_t continueWithLastPosition() {
  return lastPosition;
}

// פונקציה שמחזירה ערך מיקום שגורם לרובוט לנטות שמאלה מהמרכז
uint16_t driftLeftFromCenter() {
  return 2000; // ערך קטן מהמרכז (2000) יגרום לשגיאה חיובית ולפנייה שמאלה
}

// פונקציה שקוראת את חיישני הקו ומחזירה את המיקום המשוקלל של הקו השחור
uint16_t readLine() {
  uint16_t position = qtr.readLineBlack(sensorValues); // קריאה של ערכי החיישנים וחישוב מיקום הקו השחור
  onLine = false; // איפוס הדגל שאומר אם הרובוט רואה קו
  bool allOnBlack = true; // דגל שאומר אם כל החיישנים רואים שחור

  // בדיקה אם לפחות אחד החיישנים רואה קו שחור
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > threshold[i]) { // אם ערך החיישן גבוה מהסף, הוא רואה שחור
      onLine = true; // עדכון הדגל - הרובוט רואה קו
    } else {
      allOnBlack = false; // אם חיישן לא רואה שחור, הדגל מתעדכן
    }
  }

  // אם הרובוט לא רואה קו שחור כלל
  if (!onLine) {
    // אם המיקום האחרון שהרובוט זיהה היה במרכז
    if (lastPosition<2000||lastPosition>-2000) {
      return driftLeftFromCenter(); // החזרת ערך שיגרום לרובוט לנטות שמאלה
    } else {
      return continueWithLastPosition(); // החזרת המיקום האחרון כדי שהרובוט ימשיך בכיוון האחרון
    }
  }

  // אם כל החיישנים רואים שחור, החזר את המיקום האחרון
  if (allOnBlack) {
    return continueWithLastPosition();
  }

  lastPosition = position; // עדכון המיקום האחרון של הקו השחור
  return position; // החזרת המיקום הנוכחי של הקו השחור
}

// פונקציה המבצעת בקרת PID כדי לחשב את תיקון המהירות של המנועים
void PID_Linefollow(int error) {
  P = error; // הרכיב הפרופורציונלי הוא השגיאה הנוכחית
  I += error; // הרכיב האינטגרלי הוא סכום השגיאות לאורך זמן
  D = error - previousError; // הרכיב הנגזר הוא השינוי בשגיאה

  // חישוב ערכי ה-P, I ו-D לאחר חלוקה בקבוע מתאים
  Pvalue = (Kp / pow(10, multiP)) * P;
  Ivalue = (Ki / pow(10, multiI)) * I;
  Dvalue = (Kd / pow(10, multiD)) * D;

  PIDvalue = Pvalue + Ivalue + Dvalue; // חישוב ערך ה-PID הסופי

  previousError = error; // עדכון השגיאה הקודמת לחישוב הנגזר בפעם הבאה

  // חישוב מהירויות המנועים החדשות על סמך ערך ה-PID
  lsp = BASE_SPEED - PIDvalue; // הפחתת ערך ה-PID מהמהירות הבסיסית של המנוע השמאלי
  rsp = BASE_SPEED + PIDvalue; // הוספת ערך ה-PID למהירות הבסיסית של המנוע הימני

  motor_drive(lsp, rsp); // קריאה לפונקציה שמפעילה את המנועים עם המהירויות החדשות
}

// פונקציה השולטת על מהירות וכיוון המנועים
void motor_drive(int left, int right) {
  // הגבלת המהירות למקסימום המוגדר
  left = constrain(left, -MAX_SPEED, MAX_SPEED);
  right = constrain(right, -MAX_SPEED, MAX_SPEED);

  // שליטה במנוע שמאלי
  analogWrite(PWMA, abs(left)); // שליטה על מהירות המנוע השמאלי באמצעות PWM (ערך מוחלט של המהירות)
  digitalWrite(AIN1, left > 0 ? HIGH : LOW); // קביעת כיוון המנוע השמאלי (קדימה אם left חיובי)
  digitalWrite(AIN2, left < 0 ? HIGH : LOW); // קביעת כיוון המנוע השמאלי (אחורה אם left שלילי)

  // שליטה במנוע ימני
  analogWrite(PWMB, abs(right)); // שליטה על מהירות המנוע הימני באמצעות PWM (ערך מוחלט של המהירות)
  digitalWrite(BIN1, right > 0 ? HIGH : LOW); // קביעת כיוון המנוע הימני (קדימה אם right חיובי)
  digitalWrite(BIN2, right < 0 ? HIGH : LOW); // קביעת כיוון המנוע הימני (אחורה אם right שלילי)
}