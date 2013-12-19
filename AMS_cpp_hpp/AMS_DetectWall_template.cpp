/// Beuth Hochschule für Technik Berlin
/// Autonome Mobile Systeme
/// Prof. Dr.-Ing. Volker Sommer

#include "AMS_Robot.hpp"
#include "newmat/newmatio.h" // Input/output für Newmat

using namespace AMS;
using namespace PlayerCc;
using namespace std;
namespace po = boost::program_options;

AMS_Robot robot;

int main(int argc, char **argv)
{
    double *scan;                   // Zeiger auf aktuellen Scan
    int LUT[360];                   // Look-Up-Table mit Indizes zu allen relevanten Messwerten
    double MaxRange;                // Maximale Messdistanz des Entfernungssensors in m
    double sigma_r;                 // Standardabweichung des Entfernungssensors in m
    int N=0;                        // Anzahl der relevanten Messungen im Scan
    double Num=0, Num1;             // Zählersumme und -zwischensumme zur Berechnung von Phi
    double Den=0, Den1;             // Nennersumme und -zwischensumme zur Berechnung von Phi
    double Phi;                     // Winkel der Regressionsgeraden
    double Phi1;                    // Erster möglicher Wert für Phi
    double Phi2;                    // Zweiter möglicher Wert für Phi
    double d;                       // Normalenabstand der Regressionsgeraden von der Roboterposition
    double d1=0;                    // Wert für d, falls Phi = Phi1
    double d2=0;                    // Wert für d, falls Phi = Phi2
    double var_rho;                 // Varianz des Abstands der Messpunkte von der Regressionsgeraden
    double var_rho1=0;              // Wert der Varianz, falls d = d1 und Phi = Phi1
    double var_rho2=0;              // Wert der Varianz, falls d = d2 und Phi = Phi2
    int i,j;                        // Laufvariablen, gleichzeitig Winkel in deg
    double x, y, theta;             // Roboterkoordinaten

    // Roboter initialisieren
	if( !(robot.read_config(argc, argv) && robot.connect()) ) {
		robot.log.notice("Call with -h to see the available options.");
		return -1;
	}

    robot.init_push_mode();  // Daten von "robot" aus Warteschlange lesen
    robot.wait_for_new_data(); // Daten aktualisieren

    robot.get_scan(scan, MaxRange, sigma_r); // Messwerte, max. Messdistanz und radiale Standardabweichung auslesen

    /********************* Fügen Sie ab hier eigenen Quellcode ein **********************/

    // Bestimmung der Anzahl der relevanten Messwerte und Besetzen der Look-Up-Table

    // Rücksprung, falls keine Messwerte vorhanden sind

    // Berechnung des Winkels der Regressionsgeraden (zwei Alternativen für Phi)

    // Berechnung der Abstände der Regressionsgeraden für beide Alternativen

    // Berechnung der mittleren quadratischen Fehler für die beiden Alternativen

    // Auswahl einer der beiden Alternativen aufgrund des kleineren Fehlers

    // Globale Koordinaten der gefundenen linearen Kontur bestimmen

    // Korrektur bei negativem Normalenabständen


    /******************** Ende des zusätzlich eingefügten Quellcodes ********************/

    while(1); // Endlosschleife
    return 0;
}
