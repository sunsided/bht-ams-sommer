/// Beuth Hochschule für Technik Berlin
/// Autonome Mobile Systeme
/// Prof. Dr.-Ing. Volker Sommer

#include "AMS_Robot.hpp"
#include "AMS_WallMap.hpp"

using namespace PlayerCc;
using namespace std;

// Im Konstruktor: Hinderniskonturen einlesen und in Vektor-Attribut speichern
WallMap::WallMap() {
        ifstream infile;
        wall walli;
        infile.open("AMS_Walls.txt");
        while( infile >> walli.phi >> walli.dist >> walli.Q11 >> walli.Q22 >> walli.Q12 ) {
            Walls.push_back(walli);
        }
        infile.close();
    }
void WallMap::out() {
        uint i;
        for( i=0; i<Walls.size(); i++ )
            printf("%3d: phi=%4.0lf  dist=%5.2lf\n", i+1, Walls.at(i).phi, Walls.at(i).dist);
    }
// In search(): Suchen nach gespeicherter Kontur mit minimalem Abstand zu übergebener Geraden
bool WallMap::search(int& phi, double& dist, int delta_phi_max, double delta_dist_max) {
        uint32_t i;
        double phi_temp = 1000;
        double dist_temp = 1000;
        double delta_phi_temp = dtor(delta_phi_max);
        double delta_dist_temp = delta_dist_max;

        for( i=0; i<Walls.size(); i++ ) {
            if( fabs(normalize(dtor(phi-Walls.at(i).phi)))<=delta_phi_temp && fabs(dist-Walls.at(i).dist)<=delta_dist_temp) {
                phi_temp = Walls.at(i).phi;
                dist_temp = Walls.at(i).dist;
                delta_phi_temp = fabs(normalize(dtor(phi-Walls.at(i).phi)));
                delta_dist_temp = fabs(dist-Walls.at(i).dist);
            }
        }

        if( phi_temp==1000 && dist_temp==1000 )
            return false;

        phi = phi_temp;
        dist = dist_temp;

        return true;
    }

