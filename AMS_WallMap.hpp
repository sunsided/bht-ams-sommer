/// Beuth Hochschule für Technik Berlin
/// Autonome Mobile Systeme
/// Prof. Dr.-Ing. Volker Sommer

// Hilfsklasse zum Laden der Konturen in "AMS_Walls.txt" in einen Vektor und zum Suchen darin nach aktueller Kontur
#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

struct wall {
    double phi;
    double dist;
    double Q11;
    double Q22;
    double Q12;
};

class WallMap {
private:
    vector<wall> Walls;
public:
    WallMap();      // Konstruktor mit Initialisierung
    void out();     // Ausgabe sämtlicher gespeicherten Wandkonturen
    // Suche nach einer gespeicherten Kontur mit möglichst geringer Abweichung zu den übergebenden
    // Parametern phi und dist
    /// @param phi [in/out] übergibt den aktuellen Winkel und gibt den gefundenen Winkel zurück
    /// @param dist [in/out] übergibt den aktuellen Abstand und gibt den gefundenen Abstand zurück
    /// @param delta_phi_max [in] definiert den Betrag der maximal erlaubte Winkelabweichung in [Grad]
    /// @param delta_dist_max [in] definiert den Betrag der maximal zulässigen Abstandsabweichung in [m]
	/// @returns "true", falls Gerade gefunden wurde, sonst "false"
    bool search(int& phi, double& dist, int delta_phi_max, double delta_dist_max);
};
