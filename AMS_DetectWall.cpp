/// Beuth Hochschule für Technik Berlin
/// Autonome Mobile Systeme
/// Prof. Dr.-Ing. Volker Sommer

#include <iostream>
#include <cmath>

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
    cout << "Auswertung Messwerte ..." << endl;

    N = 0;
    for (i=0; i<360; ++i)
    {
      if (scan[i] <= 0) continue;
      LUT[N++] = i;
    }

    // Rücksprung, falls keine Messwerte vorhanden sind
    cout << "Anzahl Messwerte: " << N << endl;
    if (N == 0) return 0;

    // Berechnung des Winkels der Regressionsgeraden (zwei Alternativen für Phi)
    Num = Den = 0;
    for (i=0; i<N; ++i)
    {
      const int index_i = LUT[i];
      const double theta_i = index_i*M_PI/180.0;
      const double r_i = scan[index_i];

      Num1 = 0;
      Den1 = 0;

      for (j=0; j<N; ++j)
      {
        const int index_j = LUT[j];
        const double theta_j = index_j*M_PI/180.0;
        const double r_j = scan[index_j];

        Num1 += r_i*r_j * cos(theta_i) * sin(theta_j);
        Den1 += r_i*r_j * cos(theta_i + theta_j);
      }

      Num1 *= 2/N;
      Den1 *= 1/N;

      Num += r_i*r_i * sin(2 * theta_i) - Num1;
      Den += r_i*r_i * cos(2 * theta_i) - Den1;
    }

    Phi1 = 0.5*atan2(Den, Num);
    Phi2 = Phi1 + 0.5*M_PI;

    cout  << endl << "Ermittelte Werte für Phi:" << endl
          << "Phi1: " << Phi1 << " (" << (Phi1*180/M_PI) << "°)" << endl
          << "Phi2: " << Phi2 << " (" << (Phi2*180/M_PI) << "°)" << endl;

    // Berechnung der Abstände der Regressionsgeraden für beide Alternativen
    d1 = d2 = 0;
    for (i=0; i<N; ++i)
    {
      const int index = LUT[i];
      const double theta_i = index*M_PI/180.0;
      const double r_i = scan[index];

      d1 += r_i * cos(theta_i - Phi1);
      d2 += r_i * cos(theta_i - Phi2);
    }

    d1 /= N;
    d2 /= N;

    cout  << endl << "Ermittelte Werte für d:" << endl
          << "d1: " << d1 << endl
          << "d2: " << d2 << endl;

    // Berechnung der mittleren quadratischen Fehler für die beiden Alternativen
    var_rho1 = var_rho2 = 0;
    for (i=0; i<N; ++i)
    {
      const int index = LUT[i];
      const double theta_i = index*M_PI/180.0;
      const double r_i = scan[index];

      double temp = r_i * cos(theta_i - Phi1) - d1;
      var_rho1 += temp*temp;

      temp = r_i * cos(theta_i - Phi2) - d2;
      var_rho2 += temp*temp;
    }

    var_rho1 /= N; // überflüssig
    var_rho2 /= N; // überflüssig

    cout  << endl << "Ermittelte Varianzen:" << endl
          << "rho1: " << var_rho1 << endl
          << "rho2: " << var_rho2 << endl;

    // Auswahl einer der beiden Alternativen aufgrund des kleineren Fehlers
    var_rho = var_rho1;
    d = d1;
    Phi = Phi1;

    if (var_rho2 < var_rho)
    {
      var_rho = var_rho2;
      d = d2;
      Phi = Phi2;
    }

    cout  << endl << "Ermittelte Geradenparameter:" << endl
          << "Phi: " << Phi << " (" << (Phi*180/M_PI) << "°)" << endl
          << "d:   " << d << endl
          << "rho: " << var_rho << endl;

    // Globale Koordinaten der gefundenen linearen Kontur bestimmen
    x = robot.get_x();
    y = robot.get_y();
    theta = robot.get_theta();

    cout  << endl << "Roboterposition:" << endl
          << "x: " << x << endl
          << "y: " << y << endl
          << "theta: " << theta << endl;

    const double beta = atan2(y, x);
    double global_phi = theta + Phi;
    double global_d   = d + sqrt(x*x + y*y) * cos(beta - theta);

    // Korrektur bei negativem Normalenabständen
    if (d < 0)
    {
      global_phi += M_PI;
      d = fabs(d);
    }

    cout  << endl << "Globale Koordinaten der Wand:" << endl
          << "phi: " << global_phi << endl
          << "d:   " << global_d << endl;

    /******************** Ende des zusätzlich eingefügten Quellcodes ********************/

    while(1); // Endlosschleife
    return 0;
}
