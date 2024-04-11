#include "common.h"
#include "ipddp.h"
#include "road_data.h"
#include "dynamic_car.h"
#include <random>

int main() {
    
    for(int index_data=1; index_data<=100; index_data++) {

        IPDDP* ipddp;
        VecOfVecXd u0;
        VectorXd x0;

        // make dynamic_car
        Model* dynamic_car = new DynamicCar();

        // define problem
        double dt = 0.15;
        ipddp = new IPDDP(dynamic_car, dt);

        // define initial state[0.0, 0.0, 0.0, 0.0]; target state [65.0，-10.0, 0.0, *]
        x0.resize(4);
        x0 << 2.0, 2.0, 0.0, 0.0;

        // make initialization for control sequence
        int T = 40;
        VectorXd u_init(1); 
        for (int i=0; i<T; i++) {
            u_init << 1;
            //u_init = Vector2d::Random();
            u0.push_back(0.00*u_init);
        }

        std::vector<std::vector<double> > road_data;
        road_data = load_data(index_data);
        //print_road_date(road_date);

        //cout << "horizon = " << T << endl;
        // Solve
        //cout << "Run IPDDP!" << endl;
        auto start = std::chrono::system_clock::now();
        ipddp->generate_trajectory(x0, u0, road_data, index_data);
        auto now = std::chrono::system_clock::now();
        long int elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
        cout << "IPDDP took: " << elapsed/1000. << "seconds." << endl;
    }    

    return 0;
}