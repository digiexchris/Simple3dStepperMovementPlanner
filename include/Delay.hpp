#pragma once

#include <algorithm>
#include <cmath>

constexpr double PI = 3.14159265358979323846;

/**
    @brief Calculate the time interval between the previous step and this step
    @param stepsPerRotation The number of steps per rotation of the stepper
   motor
    @param acceleration The acceleration of the stepper motor in steps per
   second squared
    @param previousInterval The previous interval in seconds
    @return The next interval in seconds
 */
inline double calculateNextInterval(int stepsPerRotation, double acceleration,
                                    double previousInterval) {
  double alpha = 2 * PI / stepsPerRotation; // Step angle in radians

  // Convert acceleration from steps/s^2 to rad/s^2
  double radsPerSecondSquared = acceleration * alpha;

  // If previousInterval is hard-coded, assume first interval is provided
  // correctly
  if (previousInterval <= 0) {
    return std::sqrt(2 * alpha /
                     acceleration); // Use original acceleration in steps/s^2
  }

  // Calculate subsequent intervals
  return previousInterval -
         (2 * previousInterval * previousInterval * radsPerSecondSquared) /
             alpha;
}

// #define K 200   // number of steps per one revolution
// #define F 10000 // interrupt frequency 10kHz
// //
// float s = 2 * PI, vs = 0.5, vM = 2, aM = 2.0, m = 2, T, ta, tv, td, ss, aa =
// aM,
//       vv = vM; // a[rad/sec^2],v[rad/sec],s[rad]
// double alfa = TWO_PI / K;
// double Af = aM / alfa / F / F;
// double Vsf = vs / alfa / F;
// double Df = -Af / m; // Af[steps/Ts^2],Vsf[steps/Ts]
// double brzina = Vsf;
// double pozicija = 0; //
// // cN-current number of steps, dN-desired number of steps
// long N, Na, Nv, Nd, broj;
// long cN;
// long np = -1;
// byte cw[] = {0b0001,  // A,
//              0b0011,  // A,B
//              0b0010,  // B,
//              0b0110,  // B,C
//              0b0100,  // C,
//              0b1100,  // C,D
//              0b1000,  // D,
//              0b1001}; // D,A
// byte dcw = sizeof(cw), kk = 0, maska = 0xF0, ledPin = 13;
// char ch, chh;
// boolean smjer = true, stoj = true;
// //
// void loop() {
//   if (np >= Na && np < N - Nd)
//     Af = 0; // speed is constant
//   if (np >= N - Nd)
//     Af = Df; // begin of deceleration
//   if (np >= N) {
//     stoj = true;
//     Serial.print("..............................................\n");
//     Serial.print("STEPS = ");
//     Serial.print(cN);
//     Serial.print(" [steps]\n");
//     Serial.print("ANGLE = ");
//     Serial.print(cN * 360.0 / K, 2);
//     Serial.print(" [deg]\n");
//     Serial.print("POSITION = ");
//     Serial.print(float(cN * alfa), 2);
//     Serial.print(" [rad]\n");
//     np = -1;
//     pozicija = 0;
//     brzina = Vsf;
//     digitalWrite(ledPin, LOW);
//   }
// }
// // this procedure enables the entry of new data
// void serialEvent() {
//   if (Serial.available() > 0) {
//     ch = toupper(Serial.read());
//     if (ch == 'P' || ch == 'B' || ch == 'V' || ch == 'A' || ch == 'W' ||
//         ch == 'R' || ch == 'S' || ch == 'M')
//       chh = ch;
//     if (ch >= '0' && ch <= '9')
//       broj = broj * 10 + int(ch - '0');
//     if (ch == ',') {
//       if (chh == 'P') {
//         s = broj / 10.0; // s[deg]
//         stoj = planTrajek(s);
//       }
//       if (chh == 'B')
//         vs = broj / 10.0;
//       if (chh == 'V')
//         vM = broj / 10.0;
//       if (chh == 'A')
//         aM = broj / 10.0;
//       if (chh == 'R')
//         m = broj / 10.0;
//       if (chh == 'S')
//         stoj = true;
//       if (chh == 'M')
//         stoj = false;
//       if (chh == 'W')
//         pisiSve();
//       broj = 0;
//     }
//   }
// }
// // trajectory planning procedure
// boolean planTrajek(float _s) {
//   Serial.print("\nDATA: newP = ");
//   Serial.print(_s, 2);
//   Serial.print("[deg], ");
//   Serial.print(", curP = ");
//   Serial.print(float(cN * 360.0 / K));
//   Serial.print("[deg]");
//   //
//   N = int(_s * K / 360.0) -
//       cN; // difference from desired and current position [steps]
//   if (N < 0)
//     smjer = false;
//   else
//     smjer = true;
//   if (N == 0) {
//     np = -1;
//     return true;
//   } //
//   //
//   N = abs(N);
//   float s_ = N * alfa;
//   T = sqrt(pow((vs * (1 + m) / aa), 2) + 2 * s_ * (1 + m) / aa) -
//       vs * (1 + m) / aa; // N = round(s/alfa);
//   ta = T / (1 + m);
//   vv = vs + aa * ta; // td = (T-ta); tv = 0;
//   if (vv <= vM) {    // TRIANGULAR PROFILE
//     Na = int(N / (1 + m));
//     Nd = N - Na;
//     Nv = 0;
//   } else { // TRAPEZOIDAL PROFILE
//     vv = vM;
//     ta = (vM - vs) / aa;
//     T = (1 + m) * ta;
//     ss = vs * T + aa * pow(T, 2) / (2 * (1 + m));
//     // ss - is tha part of movement in the phases of acceleration and
//     // deceleration only np - number of steps on the movement ss
//     np = int(ss / alfa);
//     T = (1 + m) * ta + (s_ - ss) / vv; // tv = (s_-ss)/vv; td = T - ta - tv;
//     Na = int(np / (m + 1));
//     Nd = np - Na;
//     Nv = N - np; // Nv - number of steps in the phase v=const.
//   }
//   // Vsf [steps/0.1ms] - the start speed, Af[steps/(0.1ms)^2]-acceleration,
//   // Df-deceleration
//   Vsf = vs / alfa / F;
//   Af = aa / alfa / F / F;
//   Df = -Af / m;
//   brzina = Vsf;
//   pozicija = 0;
//   np = 0;
//   13 1234567890 International Conference on Applied Sciences(ICAS2017)
//               IOP Publishing IOP Conf.Series
//       : Materials Science and Engineering 294(2017)012055 doi : 10.1088 /
//           1757 -
//       899X / 294 / 1 / 012055 Serial.print(", dN = ");
//   Serial.print(N);
//   Serial.print(", Direction = ");
//   Serial.print((smjer) ? "CW.\n" : "CCW.\n");
//   digitalWrite(ledPin, HIGH);
//   return false;
//   //
// }