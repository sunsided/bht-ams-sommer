/// Beuth Hochschule für Technik Berlin
/// Autonome Mobile Systeme
/// Prof. Dr.-Ing. Volker Sommer

#include "AMS_KalmanFilter.hpp"
#include <iostream>
#include "newmat/newmatio.h"

using namespace AMS;
using namespace PlayerCc;
using namespace std;

KalmanFilter::KalmanFilter(AMS_Robot* robotpointer)
{
    // Dimension von P festlegen und P initialisieren
    P.ReSize(3,3);
    P = 0.0;
    // Zeiger auf Roboterobjekt als Attribut speichern
    this->robotp = robotpointer;
    b = 0.27;     // Abstand der Räder vom kinematischen Zentrum des Roboters [m]
    ks = 0.001;   // Schlupfkonstante zur Berechnung der Varianz der Roboterbewegung in [m]
    red = 0;
    green = 255;
    blue = 0;
    // Kinematik initialisieren
    D.ReSize(2,2);  // Dimension festlegen
    D << 0.5   <<  0.5
      << 0.5/b << -0.5/b;
    // Simulierten Schlupffaktor des Roboters setzen
    robotp->set_slip_const(0.001);
    // Simulierten Messfehler des Entfernungssensors setzen (erst für Übungsaufgabe 8 relevant)
    robotp->set_sigma_ranger(0.04);
}

void KalmanFilter::PlotEllipse(double xm, double ym)
{
    int pt_count = 360;        // Anzahl der Ellipsenpunkte
    double alpha;              // Parameter zum Zeichnen der Ellipse
    SymmetricMatrix P1(2);     // Kopie von P als symmetrische Matrix für Eigenwertberechnung
    Matrix T(2,2);             // Matrix mit den othogonalen Eigenvektoren von P
    DiagonalMatrix L(2);       // Diagonalmatrix mit den Eigenwerten von P
    ColumnVector xys(2);       // Vektor mit jeweils aktuellem Ellipsenpunkt (xs,ys) in Hauptachsenform (lokale Koordinaten)
    ColumnVector xy(2);        // Vektor mit jeweils aktuellem Ellipsenpunkt (x,y) in globalen Koordinaten
	  player_point_2d_t* ellipse = new player_point_2d_t[pt_count+1]; // Objekt zum Speichern der Ellipsenpunkte

    /********************* Fügen Sie ab hier eigenen Quellcode ein **********************/

    cout << "== ELLIPSE ==================================" << endl;

    // Kovarianzen für x und y extrahieren
    cout << "Extrahiere Kovarianzen" << endl;
    P1(1,1) = P(1,1);
    P1(1,2) = P(1,2);
    P1(2,2) = P(2,2);

    cout << P1 << endl;

    // Eigentwerte der Kovarianzen von x und y ermitteln
    eigenvalues(P1, L, T);
    const double a = L(1);
    const double b = L(2);

    cout << "Längen der Hauptachsen" << endl
         << "a = " << a << endl
         << "b = " << b << endl;

    // Schleife zur Berechnung der Ellipse in Parameterform und zum Speichern im Array "ellipse"
    const double alpha_start = 0;
    const double alpha_end = 2*M_PI;
    const double alpha_inc = (alpha_end-alpha_start) / pt_count;
    for (int i = 0; i < pt_count; ++i)
    {
      // Ellipse in lokalen Koordinaten berechnen
      alpha = alpha_start + i*alpha_inc;
      xys << a * cos(alpha)
          << b * sin(alpha);

      // Transformation in globale Koordinaten
      xy << T*xys;
      xy(1) += xm;
      xy(2) += ym;

      // Koordinaten in Ellipse übertragen
      player_point_2d_t point = { xy(1), xy(2) };
      ellipse[i] = point;
    }

    ellipse[pt_count] = ellipse[0];

    /******************** Ende des zusätzlich eingefügten Quellcodes ********************/

    robotp->graphmap->Color(red,green,blue,0);  // Farben im Roboterobjekt setzen
	  robotp->graphmap->DrawPolyline(ellipse, pt_count+1); // Ellipse zeichnen
}

void KalmanFilter::PredictCov(double theta, double delta, double phi)
{
    ColumnVector u(2);          // Eingangsvektor mit delta und phi
    ColumnVector u_rl(2);       // Eingangsvektor mit Weginkrementen für rechtes und linkes Rad
    Matrix A(3,3);              // Systemmatrix des linearisierten Zustandsmodells
    Matrix B(3,2);              // Eingangsmatrix des linearisierten Zustandsmodells
    DiagonalMatrix Q_rl(2);     // Kovarianzmatrix für Wegstrecken des rechten und linken Rades
    Matrix Q(2,2);              // Kovarianzmatrix für Eingangsdaten delta und phi

    /********************* Fügen Sie ab hier eigenen Quellcode ein **********************/

    cout << "== PRÄDIKTION ===============================" << endl;

    // Eingangsgrößen in Vektor speichern
    u  << delta
       << phi;

    // Rückrechnung von delta und phi auf Wegstrecken der beiden Räder
    cout << "Invertierung von D" << endl;
    InvertedMatrix invertedD = D.i();
    u_rl << invertedD * u;

    cout << setw(12) << setprecision(5) << invertedD << endl;

    // Varianzinkrement der Räder proportional zu Radwegen und abhängig von ks vorgeben
    cout << "Berechnung von Q_rl" << endl;
    Q_rl(1,1) = u_rl(1) * ks;
    Q_rl(2,2) = u_rl(2) * ks;

    cout << setw(12) << setprecision(5) << Q_rl << endl;

    // Varianzinkrement für delta und phi berechnen
    cout << "Berechnung von Q" << endl;
    Q << D * Q_rl * D.t();

    cout << setw(12) << setprecision(5) << Q << endl;

    // System- und Eingangsmatrix für aktuellen Schritt bestimmen
    cout << "Berechnung von A" << endl;
    A << 1 << 0 << -delta * sin(theta + phi/2)
      << 0 << 1 <<  delta * cos(theta + phi/2)
      << 0 << 0 << 1;

    cout << setw(12) << setprecision(5) << A << endl;

    cout << "Berechnung von B" << endl;
    B << cos(theta + phi/2) << -delta/2 * sin(theta + phi/2)
      << sin(theta + phi/2) <<  delta/2 * cos(theta + phi/2)
      << 0 << 1;

  cout << setw(12) << setprecision(5) << B << endl;

    // Prädiktion der Kovarianzmatrix
    cout << "Berechnung von P" << endl;
    P << A*P*A.t() + B*Q*B.t();

    /******************** Ende des zusätzlich eingefügten Quellcodes ********************/

    // Ausgabe der Kovarianzmatrix
    cout << setw(12) << setprecision(5) << P << endl;
}

/// Die folgende Methode ist erst für Übungsaufgabe 8 relevant
#if 0
bool KalmanFilter::Correction(double& x, double& y, double& theta)
{
    double PhiR;               // Normalenwinkel einer erkannten Wand relativ zum Roboter
    double dR;                 // Normalenabstand einer erkannten Wand vom Roboter
    int Phi;                    // globale Ausrichtung der aktuell erfassten Geraden
    double d;                  // globaler Abstand der aktuellen Geraden
    SymmetricMatrix R(2);       // Kovarianzmatrix der aktuellen Messung
    Matrix H(2,3);              // Messmatrix
    ColumnVector z(2);          // Messvektor mit x_phi und theta
    Matrix K(3,2);              // Kalman-Gain Matrix
    IdentityMatrix E(3);        // Einheitsmatrix
    ColumnVector x_(3);         // Vektor für Roboterzustand

    // Auswertung des Scans und Rücksprung, falls keine Wand detektiert wurde
    if( !robotp->detect_wall(PhiR, dR, R) ) {
        printf("Keine Wand gefunden\n");
        return false;
    }

    //********************* Fügen Sie ab hier eigenen Quellcode ein **********************

    // Globale Koordinaten der gefundenen Kontur bestimmen für Suche in Karte

    // Suchen nach der aktuell gefundenen Geraden in der Karte; Rücksprung, falls keine Übereinstimmung

    // Bestimmung der Messdaten für den Korrekturschritt
    z(1) = ;
    z(2) = ;

    // Alternative Bestimmung der Messdaten, falls gemessene Kontur zwischen Roboter und Koordinatenursprung liegt
    // Dies kann aufgrund einer zu großer Abweichung (> 0.5 rad) zwischen gemessenem und prädiziertem theta erkannt

    // Messmatrix abhängig von der Ausrichtung der Kontur festlegen

    // Kalman Gain für aktuellen Schritt berechnen
    K = ;
    // Zustand mit Messung korrigieren; vorher Roboterzustand in Vektor x_ speichern
    x_ = ;
    // Kovarianzmatrix korrigieren
    P =;

    //******************** Ende des zusätzlich eingefügten Quellcodes ********************

    return true;
}
#endif
