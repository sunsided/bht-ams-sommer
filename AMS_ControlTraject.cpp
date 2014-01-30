/// Beuth Hochschule für Technik Berlin
/// Autonome Mobile Systeme
/// Prof. Dr.-Ing. Volker Sommer

#include "AMS_Robot.hpp"
#include "boost/date_time/posix_time/posix_time.hpp" // Für Zeitmessungen
#include <fstream> // Für Datei-Zugriff
#include <cmath>
#include <iostream>
#include <iomanip>

using namespace std;
using namespace AMS;
using namespace PlayerCc;

AMS_Robot robot;

int main(int argc, char **argv)
{
    const double T = 0.25; // feste Zeitdauer zwischen zwei Wegpunkten in Sekunden
    const double invT = 1.0 / T;
    double v, w;           // aktuelle Bahn- und Winkelgeschwindigkeit
    double delta, phi;     // im aktuellen Schritt zurückgelegte Distanz und überstrichener Drehwinkel
    double vx, vy;         // aktuelle x- und y-Komponenten der Bahngeschwindigkeit in Schritt i
    double x, y;           // aktuelle Position in Schritt i
    double theta;          // aktueller Ist-Winkel in Schritt i
    double v_s=0, w_s=0;   // Stellgrößen für Bahn- und Winkelgeschwindigkeit
    double x_s, y_s;       // Soll-Koordinaten im aktuellen Schritt i
    double x_s1, y_s1;     // Soll-Koordinaten im vorherigen Schritt i-1
    double vx_s, vy_s;     // Soll-Bahngeschwindigkeit in Schritt i
    double vx_s1, vy_s1;   // Soll-Bahngeschwindigkeit in Schritt i-1
    double ax_s=0, ay_s=0; // Soll-Bahnbeschleunigung in Schritt i
    double ax, ay;         // Regelgrößen in Schritt i
    double a1, a2;         // Zwischengrößen für Regelung im Schritt i
    double e_x, e_y;       // Fehler zwischen Soll- und Ist-koordinaten
    double e_vx, e_vy;     // Fehler zwischen Soll- und Ist-Geschwindigkeiten
    const double kI=5.0;   // Regelparameter für Geschwindigkeitsfehler (entspricht P-Anteil)
    const double kII=1.0;  // Regelparameter für Positionsfehler (entspricht I-Anteil)
    string file;           // Datei mit Punkten der zu befahrenden Trajektorie
    double dt;             // Zeitdauer für das Durchlaufen der aktuellen Schleife
    ptime tref;            // Objekt der Klasse ptime zur Messung von dt

    // Roboter initialisieren
    if( !(robot.read_config(argc, argv) && robot.connect()) ) {
      robot.log.notice("Call with -h to see the available options.");
      return -1;
    }

    // Pfad zu Datei mit vorgegebener Trajektorie
    file = "AMS_resources/AMS_Trajekt.txt";

    robot.init_pull_mode();  // Nach Wartezeit immer aktuelle Werte aus Proxies lesen

	  robot.draw_traject(file, 255, 0, 0);   // Trajektorie plotten
    ifstream infile(file.c_str()); // Datei mit Trajektorie öffnen
    infile >> x_s >> y_s;      // Soll-Koordinaten des Startpunktes lesen
    infile >> x_s1 >> y_s1;        // Soll-Koordinaten des zweiten Punktes lesen

    // Position des Roboters initialisieren
    x = x_s;
    y = y_s;

    cout << fixed << showpoint;
    cout << setprecision(4);

    /********************* Fügen Sie ab hier eigenen Quellcode ein **********************/
    const double dx = x_s1 - x_s;
    const double dy = y_s1 - y_s;

    // Winkel vom 1. zum 2. Punkt der Trajektorie berechnen
    theta = atan2(dy, dx);

    // Soll-Startgeschwindigkeiten ermitteln
    vx_s1 = dx * invT;
    vy_s1 = dy * invT;

    /******************** Ende des zusätzlich eingefügten Quellcodes ********************/

    while( infile >> x_s >> y_s ) { // Schleife über alle Punkte der Trajektorie

        tref = microsec_clock::local_time(); // Referenzzeit zum Messen der Schleifendurchlaufzeit dt

        robot.wait_for_new_data(); // Neue Daten aus Proxies einlesen

        /********************* Fügen Sie ab hier eigenen Quellcode ein **********************/

        //  Aktuelle Bahn- und Winkelgeschwindigkeit auslesen und daraus das Weg- und Winkelimkrement ermitteln
        v = robot.get_v();
        w = robot.get_w();
        delta = v * T;
        phi   = w * T;

        // Berechnung der aktuellen Koordinaten mittels des nicht-holonomen Bewegungsmodells
        x     += delta * cos(theta + 0.5 * phi);
        y     += delta * sin(theta + 0.5 * phi);
        theta += phi;

        // Kosinus und Sinus vorberechnen
        const double sint = sin(theta);
        const double cost = cos(theta);

        // Aktuelle x- und y-Komponente der Bahngeschwindigkeit ermitteln
        vx  = v * cost;
        vy  = v * sint;

        // Soll-Geschwindigkeiten und -Beschleunigungen für aktuellen Schritt bestimmen
        vx_s = (x_s - x_s1) * invT;
        vy_s = (y_s - y_s1) * invT;
        ax_s = (vx_s - vx_s1) * invT;
        ay_s = (vy_s - vy_s1) * invT;

        // Aktuelle Fehler ermitteln
        const double enable = 1;
        e_x  = enable * (x_s - x);
        e_y  = enable * (y_s - y);
        e_vx = enable * (vx_s - vx);
        e_vy = enable * (vy_s - vy);

//        cout << "e_x: " << e_x << " e_y: " << e_y << " e_vx: " << e_vx << " e_vy: " << e_vy << endl;

        // Bestimmung und Setzen der Sollgeschwindigkeiten (Stellgrößen)
        const double ax_s = (vx_s - vx_s1) * invT;
        const double ay_s = (vy_s - vy_s1) * invT;
        ax = ax_s + kI * e_vx + kII * e_x;
        ay = ay_s + kI * e_vy + kII * e_y;
        a1 = ax *  cost + ay * sint;
        a2 = ax * -sint + ay * cost;
        v_s += a1 * T;
        w_s  = a2 / v_s;

        /******************** Ende des zusätzlich eingefügten Quellcodes ********************/

        robot.set_speed(v_s, w_s); // Einprägen der Führungsgrößen

        // Warten um insgesamt die Abtastzeit T zwischen zwei aufeinanderfolgenden Schritten
        // Dabei Abbruch, falls Berechnungszeit pro Schleifendurchlauf Abtastzeit überschreitet
        dt = (double)(microsec_clock::local_time()-tref).total_milliseconds()/1000;
        if( dt > T ) {
            printf("Fehler! Abtastzeit zu kurz: dt=%.3lf T=%.3lf\n", dt, T);
            robot.stop(); // stoppen
            while(1);     // warten
        }
        usleep((T-dt)*1000000); // Übergabe in Microsekunden

        // Soll-Werte für nächsten Schleifendurchgang umkopieren
        x_s1 = x_s;
        y_s1 = y_s;
        vx_s1 = vx_s;
        vy_s1 = vy_s;
    }

    robot.stop(); // stoppen

    cout << "Feierabend." << endl;

    while(1); // Endlosschleife
    return 0;
}
