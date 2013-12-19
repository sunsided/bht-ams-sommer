/// Beuth Hochschule für Technik Berlin
/// Autonome Mobile Systeme
/// Prof. Dr.-Ing. Volker Sommer

#include "AMS_Robot.hpp"
#include "AMS_WallMap.hpp"    // Zur Suche in bekannten Kartenkonturen nach aktuell gemessener Kontur
#include <iostream>
#include "newmat/newmatio.h"

using namespace AMS;
using namespace std;

class KalmanFilter
{
private:
    double b;                 	// Abstand der Räder vom kinematischen Zentrum des Roboters [m]
    double ks;                 	// Schlupfkonstante zur Berechnung der Varianz der Roboterbewegung in [m]
    Matrix D;                   // Matrix mit Kinematik des Differenzialantriebs
    Matrix P;                   // Kovarianzmatrix der Zustandsvariablen des Roboters
    AMS_Robot* robotp;          // Zeiger auf Roboterprojekt; wird dem Konstruktor übergeben
    uint8_t red, green, blue;   // Farbe der Fehlerellipse
    WallMap Karte;              // Objekt zum Speichern von und zur Identifikation von Wänden (erst für Aufgabe 7 relevant)

public:
    KalmanFilter(AMS_Robot* robotpointer);               	// Konstruktor
    void PlotEllipse(double xm, double ym);         		// Ausgabe der Fehlerellipse
    void PredictCov(double theta, double delta, double phi);// Prädiktion der Systemkovarianzmatrix
    bool Correction(double& x, double& y, double& theta); 	// Korrektur der Roboterposition (erst für Aufgabe 7 relevant)
};
