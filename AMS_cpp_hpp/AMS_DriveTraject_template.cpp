/// Beuth Hochschule für Technik Berlin
/// Autonome Mobile Systeme
/// Prof. Dr.-Ing. Volker Sommer

#include "AMS_Robot.hpp"
#include <fstream>

using namespace AMS;
using namespace PlayerCc;

AMS_Robot robot;

int main(int argc, char **argv)
{
    const double T = 0.25;  // feste Zeitdauer zwischen zwei Wegpunkten in Sekunden
    double x, x_1;          // aktuelle und vorherige x-Koordinate
    double y, y_1;          // aktuelle und vorherige y-Koordinate
    double ds, ds_1;        // aktueller und vorheriger Abstand zwischen benachbarten Punkten
    double theta, theta_1;  // aktueller und vorheriger Winkel zwischen benachbarten Punkten
    double dtheta;          // Differenzwinkel zwischen aktueller und vorheriger Strecke
    double kapa;            // aktuelle Krümmung
    double v, w;            // Sollwerte für Bahn- und Winkelgeschwindigkeit
    string file;            // Datei mit Punkten der zu befahrenden Trajektorie
    double dt;              // Zeitdauer für das Durchlaufen der aktuellen Schleife
    ptime tref;             // Objekt der Klasse ptime zur Messung von dt

    // Roboter initialisieren
	if( !(robot.read_config(argc, argv) && robot.connect()) ) {
		robot.log.notice("Call with -h to see the available options.");
		return -1;
	}

    /********************* Fügen Sie ab hier eigenen Quellcode ein **********************/

	// Plotten der Trajektorie in "AMS_Traject.txt" im Stage-Fenster mittels der Methode "draw_traject()"

	// Datei mit Trajektorie zum Lesen öffnen
    // Koordinaten des Startpunktes in x und y speichern
    // Koordinaten des zweiten Punktes in x_1 und y_1 speichern

    // Abstand der Punkte in ds_1 und Winkel zwischen Punkten in theta_1 speichern

    /******************** Ende des zusätzlich eingefügten Quellcodes ********************/

    while(infile >> x >> y) { // Schleife über Punkte der Trajektorie, dabei jeweils nächsten Punkt einlesen

        tref = microsec_clock::local_time(); // Referenzzeit zum Messen der Schleifendurchlaufzeit dt speichern

        /********************* Fügen Sie ab hier eigenen Quellcode ein **********************/

        // Abstand des aktuellen zum vorherigen Punkt berechnen
        ds = ;
        // Soll-Bahngeschwindigkeit bestimmen
        v = ;
        // Ausrichtung von ds ermitteln
        theta = ;
        // Differenzwinkel zum vorherigen Streckenabschnitt ermitteln, dabei Winkel auf Bereich von -pi bis pi beschränken
        dtheta = ;
        // Krümmung berechnen
        kapa = ;
        // Erforderliche Winkelgeschwindigkeit ermitteln
        w =;
        // Sollgeschwindigkeiten setzen

        // Werte für nächsten Schleifendurchgang umkopieren

        /******************** Ende des zusätzlich eingefügten Quellcodes ********************/

        // Warten um insgesamt die Zeitdauer T zwischen zwei aufeinanderfolgenden Schleifendurchläufen
        // Dabei Abbruch, falls Berechnungszeit pro Schleifendurchlauf T überschreitet
        dt = (double)(microsec_clock::local_time()-tref).total_milliseconds()/1000;
        if( dt > T ) {
            printf("Fehler! Abtastzeit zu kurz: dt=%.3lf T=%.3lf\n", dt, T);
            robot.stop(); // stoppen
            while(1);     // warten
        }
        usleep((T-dt)*1000000); // Warten in Micro-Sekunden
    }

    robot.stop(); // stoppen

    while(1); // Endlosschleife
    return 0;
}
