/// Beuth Hochschule für Technik Berlin
/// Autonome Mobile Systeme
/// Prof. Dr.-Ing. Volker Sommer

/// Verwendung:
/// Umherfahren des Roboters z. B. mit "playerjoy" durchführen
/// Dabei werden lineare Konturen automatisch erkannt und in "walls.txt" gespeichert
/// Diese Datei erhält die Ausrichtung (phi) und den Abstand (dist) jeder Geraden

#include <fstream>
#include <libplayerc++/playerc++.h>
#include <libplayerc++/utility.h>
#include "AMS_Robot.hpp"

using AMS::RangerRobot;
using namespace PlayerCc;
using namespace std;

RangerRobot robot;

struct wall {
    double  phi;          // globale Wandausrichtung in [deg]
    double  dist;         // Wandabstand vom Koordinatenursprung
};

int main(int argc, char **argv)
{
    SymmetricMatrix Q(2);
    vector<wall> Walls;
    wall leer;
    double x, y, theta; // aktueller Zustand des Roboters
    int i;              // Laufindex über alle bisher gefundenen Geraden
    int Anz = 0;        // Anzahl der bereits gefundenen Geraden
    double alpha;       // lokale Ausrichtung der aktuellen Geraden relativ zum Roboter
    double d;           // Abstand der aktuellen Geraden relativ zum Roboter
    int phi;            // globale Ausrichtung der aktuellen Geraden
    double dist;        // Abstand der aktuellen Geraden vom Koordinatenursprung
    std::ofstream outfile;
    outfile.open("AMS_Walls.txt");

	if (!(robot.read_config(argc, argv) && robot.connect())) {
		robot.log.notice("Call with -h to see the available options.");
		return -1;
	}
    robot.init_pull_mode();
    robot.set_slip_const(0); // Simulierten Schlupf des Roboter auf Null setzen
    robot.set_sigma_ranger(0); // Simulierten Messfehler des Entfernungssensors auf Null setzen

    do {
        robot.wait_for_new_data();
        if( robot.detect_wall(alpha, d, Q) ) {
            // Positionen direkt aus Player-Komponente ermitteln
            // Dies ist erforderlich, da der Client "playerjoy" die Position im Objekt "robot" nicht beeinflusst
            x = robot.drive->GetXPos();
            y = robot.drive->GetYPos();
            theta = robot.drive->GetYaw();
//            printf("x=%.3lf y=%.3lf theta=%.2lf\n", x, y, rtod(theta));
            // Globale Koordinaten der gefundenen Wand berechnen
            phi = ((int)rint(rtod(theta+alpha))+360)%360;
            dist = d + sqrt(x*x+y*y)*cos(atan2(y,x)-theta-alpha);
            if( dist < 0 ) {
                phi = (phi+180)%360;
                dist = -dist;
            }
//            printf("Wand gefunden: phi=%d dist=%.2lf\n", phi, dist);
            Anz = Walls.size();
            for( i=0; i<Anz; i++ )
                if( abs(normalize(phi-Walls.at(i).phi)) <= 1 && fabs(dist-Walls.at(i).dist) <= 0.04 )
                    break; // Abbruch, da Wand bereits bekannt ist
            if( i == Anz ) {
                printf("Neue Wand gefunden: %2d ", Anz+1);
                printf("--> phi=%3d dist=%.2lf\n", phi, dist);
                Walls.push_back(leer);
                Walls.at(Anz).phi = phi;
                Walls.at(Anz).dist = dist;
                // Speichern von phi und dist in Datei
                outfile <<setw(3)<<phi<<setw(8)<<setprecision(3)<<dist;
                // Werte der Kovarianzmatrix ebenfalls speichern
                outfile <<setw(10)<<setprecision(3)<<Q(1,1)<<setw(10)<<Q(2,2)<<setw(10)<<Q(1,2)<<std::endl;
            }
        }
        sleep(1); // 1s warten vor erneuter Suche nach einer linearen Kontur
    }
    while(1);
	return 0;;
}
