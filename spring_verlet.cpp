#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

int main() {

    // declare variables
    double m, k, x, v, t_max, dt, t, a;
    vector<double> t_list, x_list, v_list;

    // mass, spring constant, initial position and velocity
    m = 1;
    k = 1;
    x = 0;
    v = 1;
    t = 0;

    // simulation time and timestep
    t_max = 100;
    dt = 0.1;

    // get first two values to use in verlet integration
    t_list.push_back(t);
    x_list.push_back(x);
    v_list.push_back(v);
    a = -k * x / m;
    x = x + dt * v;
    v = v + dt * a;
    t = t + dt;
    t_list.push_back(t);
    x_list.push_back(x);
    v_list.push_back(v);

    // Verlet integration
    for (t = 2 * dt; t <= t_max; t = t + dt) {


        // calculate new position and velocity
        a = -k * x / m;
        x = 2 * x_list[x_list.size() - 1] - x_list[x_list.size() - 2] + a * (dt * dt);
        v = (x - x_list[x_list.size() - 2]) / (2 * dt);

        // append current state to trajectories
        t_list.push_back(t);
        x_list.push_back(x);
        v_list.push_back(v);

    }

    // Write the trajectories to file
    ofstream fout;
    fout.open("trajectories.txt");
    if (fout) { // file opened successfully
        for (int i = 0; i < t_list.size(); i = i + 1) {
            fout << t_list[i] << ' ' << x_list[i] << ' ' << v_list[i] << endl;
        }
    } else { // file did not open successfully
        cout << "Could not open trajectory file for writing" << endl;
    }
}
