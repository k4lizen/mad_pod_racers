#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <bits/stdc++.h>

using namespace std;

const double PI =  3.14159265358979323846;
const int NPODS = 2;
const double check_radius = 600;
const int steering_delta_deg = 18;
const double steering_delta = 18 * PI / 180;
const double collision_range = 800;

double wanted_dist = 4000;
int col_operation_start = -1;

bool used_boost = false;
int _time = 0;

double x[2];
double y[2];
double vx[2];
double vy[2];
double deg_angle[2];
int nc[2];

double tarx[2];
double tary[2];
double thrust[2];
double v[2];

double angle[2];
double nc_dist[2];
double nc_angle[2];
double nc_deg_angle[2];
double nc_rlx[2];
double nc_rly[2];
double v_angle[2];

double opp_x[2];
double opp_y[2];
double opp_vx[2];
double opp_vy[2];
double opp_deg_angle[2];
int opp_nc[2];

int opp_laps[2];
int opp_last_nc[2];
double opp_angle[2];

double check_x[10];
double check_y[10];
int laps;
int check_count;


double colWantedx = -1;
double colWantedy = -1;
enum ColliderState{
    GOING_WANTED,
    SLOWING_WANTED,
    WAITING,
    GOING_TARGET,
};
ColliderState colliderState = GOING_WANTED;

double next_check_x(int i){
    return check_x[(i + 1) % check_count];
}
double next_check_y(int i){
    return check_y[(i + 1) % check_count];
}
pair<double, double> next_check(int i){
    return {next_check_x(i), next_check_y(i)};
}

int sign(double x){
    return (x >= 0) - (x < 0);
}

void rotatePoint(double& rx, double& ry, double angle){ //around origin
    double s = sin(angle);
    double c = cos(angle);

    double orx = rx;
    double ory = ry;

    rx = orx * c - ory * s;
    ry = orx * s + ory * c;
}

double vector_angle(double x, double y){
    if(sign(x) == 1 && sign(y) == 1) //1. quadrant
        return atan(y / x);
    else if(sign(x) == -1 && sign(y) == 1){ //2.
        return PI + atan(y / x);
    }else if(sign(x) == -1 && sign(y) == -1){ //3.
        return PI + atan(y / x);
    }else{ //4.
        return 2 * PI + atan(y / x);
    }
}

double clamp_angle(double x){
    while(x < 0)
        x += 2 * PI;
    while(x >= 2 * PI)
        x -= 2 * PI;
    return x;
}

double angle_diff(double x, double y){
    x = clamp_angle(x);
    y = clamp_angle(y);
    double r1 = clamp_angle(x - y);
    double r2 = clamp_angle(y - x);
    if(r1 < r2){
        return -r1;
    }else{
        return r2;
    }
}

double angle_between(double x1, double y1, double x2, double y2){
    return angle_diff(vector_angle(x1, y1), vector_angle(x2, y2));
}

void printErr(double tar_angle){
    for(int i = 0; i < NPODS; ++i){
        cerr << "x, y: " << x[i] << " " << y[i] << "\n";
        cerr << "vx, vy: " << vx[i] << " " << vy[i] << "\n";
        cerr << "Angle: " << angle[i] << " " << deg_angle[i] << " Target: " << tar_angle << "\n";
        cerr << "Next: " << nc[i] << " " << check_x[nc[i]] << " " << check_y[nc[i]] << "\n";
        cerr << "\n";
    }
    cerr << "Collider State: " << colliderState << "\n";
}

double clamp_angle_deg(double x){
    while(x < 0)
        x += 360;
    while(x >= 360)
        x -= 360;
    return x;
}

const int HEIGHT = 9000;
const int WIDTH = 16000;

void turn_input(){
    for(int i = 0; i < 2; ++i){
        cin >> x[i] >> y[i] >> vx[i] >> vy[i] >> deg_angle[i] >> nc[i]; cin.ignore();
        angle[i] = deg_angle[i] * PI / 180;
        nc_rlx[i] = check_x[nc[i]] - x[i];
        nc_rly[i] = check_y[nc[i]] - y[i];
        thrust[i] = 100;
        nc_dist[i] = sqrt(nc_rlx[i] * nc_rlx[i] + nc_rly[i] * nc_rly[i]);
        nc_angle[i] = vector_angle(nc_rlx[i], nc_rly[i]);
        nc_deg_angle[i] = nc_angle[i] * 180 / PI;
        v[i] = sqrt(vx[i] * vx[i] + vy[i] * vy[i]);
        v_angle[i] = vector_angle(vx[i], vy[i]);
    }
    for(int i = 0; i < 2; ++i){
        opp_last_nc[i] = opp_nc[i];
        cin >> opp_x[i] >> opp_y[i] >> opp_vx[i] >> opp_vy[i] >> opp_deg_angle[i] >> opp_nc[i]; cin.ignore();
        opp_angle[i] = opp_deg_angle[i] * PI / 180;
        if (opp_nc[i] != opp_last_nc[i] && opp_nc[i] == 1){
            opp_laps[i]++;
        }
    }
}

double point_dist(double p1[2], double p2[2]){
    return sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]));
}

double point_dist(double p1x, double p1y, double p2x, double p2y){
    return sqrt((p1x - p2x) * (p1x - p2x) + (p1y - p2y) * (p1y - p2y));
}

double point_dist(pair<double, double> p1, pair<double, double> p2){
    return sqrt((p1.first - p2.first) * (p1.first - p2.first) + (p1.second - p2.second) * (p1.second - p2.second));
}

double simulate_step_turn(double cx, double cy, double targetx, double targety, double& cangle){
    double tangle = vector_angle(targetx - cx, targety - cy);
    double ctanglediff = angle_diff(cangle, tangle);
    //cerr << "pre-step: Angle: " << cangle * 180 / PI << " Tangle: " << tangle * 180 / PI << " Diff: " << ctanglediff << "\n";  
    if(abs(ctanglediff) <= steering_delta){
        cangle = tangle;
    }else{
        cangle += sign(ctanglediff) * steering_delta;
    }
    return cangle;
}

void simulate_step_move(int _thrust, double cangle, double& cx, double& cy, double& cvx, double& cvy){
        //facing vector * thrust
        double outx = cos(cangle) * _thrust;
        double outy = sin(cangle) * _thrust;
        cvx += outx; //changing current speed vector
        cvy += outy;
        cx = round(cx + cvx); //change position
        cy = round(cy + cvy);
        cvx = (int)(cvx * 0.85); //friction
        cvy = (int)(cvy * 0.85);
} 

//doesnt account thrust and friction
int about_to_be_hit(int i){ //-1 = no, 0 = d1, 1 = d2, 2 = my0, 3 = my1
    double mf[2] = {x[i] + vx[i], y[i] + vy[i]};
    for(int j = 0; j < 2; ++j){
        double hf[2] = {opp_x[j] + opp_vx[j],  opp_y[j] + opp_vy[j]};
        double dst = point_dist(mf, hf);
        if(dst <= 800){
            return j;
        }
    }

    double hf[2] = {x[1 - i] + vx[1 - i],  y[1 - i] + vy[1 - i]};
    double dst = point_dist(mf, hf);
    if(dst <= 800){
        return (1 - i) + 2;
    }

    return -1;
}


double opp_to_nc(int i){
    return sqrt((check_x[opp_nc[i]] - opp_x[i]) * (check_x[opp_nc[i]] - opp_x[i]) + (check_y[opp_nc[i]] - opp_y[i]) * (check_y[opp_nc[i]] - opp_y[i]));
}

double opp_to_check(int i, int j){
    double dst = opp_to_nc(i);
    for(int _c = opp_nc[i]; _c != j; _c = (_c + 1) % check_count){
        dst += point_dist(check_x[_c], check_y[_c], next_check_x(_c), next_check_y(_c));
    }
    return dst;
}

bool leading_opponent_drone(){ //false = 0, true = 1
    if (opp_laps[0] == opp_laps[1]){
        if(opp_nc[0] == opp_nc[1]){
            double dist0 = opp_to_nc(0);
            double dist1 = opp_to_nc(1);
            return dist0 > dist1; //i could check for velocity + dist but eh
        }
        int a0nc = opp_nc[0] == 0 ? check_count : opp_nc[0];  
        int a1nc = opp_nc[1] == 0 ? check_count : opp_nc[1];  
        return a0nc < a1nc;
    }
    return opp_laps[0] < opp_laps[1];
}

bool should_shield(int i){
    int j = about_to_be_hit(i);
    if(j == -1)
        return false;

    if(i == 0){ // racer
        double ad = 0;
        double tvel = 0;
        if(j > 2){ 
            j -= 2; //collider (j == 1)
            ad = angle_between(vx[j], vy[j], tarx[0], tary[0]);
            tvel = v[j];
        }else{
            ad = angle_between(opp_vx[j], opp_vy[j], tarx[0], tary[0]); //where is the collider gonna yeet me vs, where am i facing
            tvel = sqrt(opp_vx[j] * opp_vx[j] + opp_vy[j] * opp_vy[j]);
        }

        if(abs(ad * 180 / PI) <= 60)
            return false; //if he is going to push me where i want to go
        
        if(tvel < 250) //if its only a light tap
            return false;

        return true;
    }else{ //collider
        int leading = leading_opponent_drone() ? 1 : 0;
        if(j != leading){
            return false;
        }

        cerr << "About to collide with leading\n";
        return true;
    }
}

bool line_inter_circle(double x1, double y1, double x2, double y2, double xo, double yo, double r){
    x1 -= xo;
    x2 -= xo;
    y1 -= yo;
    y2 -= yo;
    double d1 = x2 - x1, d2 = y1 - y2;
    double __dr = sqrt(d1 * d1 + d2 * d2);
    double __D = x1 * y2 - x2 * y1;
    double __deter = r * r * __dr * __dr - __D * __D;
    return __deter >= 0;
}

double logab(double a, double b){
    return log2(a) / log2(b);
}

bool point_in_circle(double x, double y, double ox, double oy, double r){
    double d1 = (x - ox);
    double d2 = (y - oy);
    double dist = sqrt(d1 * d1 + d2 * d2);
    return dist <= r;
}

double angle_diff_deg(double x, double y){ // x + ? = y, returns ?
    x = clamp_angle_deg(x);
    y = clamp_angle_deg(y);
    double r1 = clamp_angle_deg(x - y);
    double r2 = clamp_angle_deg(y - x);
    if(r1 < r2){
        return -r1;
    }else{
        return r2;
    }
}

double curving_speed(double needed_angle, double speed){
    if(abs(needed_angle) * 180 / PI > 90 && speed >= 600)
        return 0;
    return 100;
}

//can i steer to the nc[i] + 1 checkpoint and still get into nc[i]
bool check_nnc_steer(){
    int csteps = 0;
    int steps = 5;

    double cx = x[0], cy = y[0];
    double nncx = next_check_x(nc[0]);
    double nncy = next_check_y(nc[0]);
    double cvx = vx[0], cvy = vy[0];
    double cangle = angle[0];

    double targetx = nncx - cvx;
    double targety = nncy - cvy;
    double tangle = vector_angle(targetx - cx, targety - cy);

    bool been_inside = false;

    double cthrust = 100;

    while(csteps < steps){
        cthrust = 100;
        targetx = nncx - cvx;
        targety = nncy - cvy;
        tangle = vector_angle(targetx - cx, targety - cy);

        double ctanglediff = angle_diff(cangle, tangle);
        //cerr << "pre-step: Angle: " << cangle * 180 / PI << " Tangle: " << tangle * 180 / PI << " Diff: " << ctanglediff << "\n";  
        if(abs(ctanglediff) <= steering_delta){
            cangle = tangle;
        }else{
            cangle += sign(ctanglediff) * steering_delta;
        }
        //lower thrust if bad angle, and moving fast
        cthrust = curving_speed(angle_diff(cangle, tangle), sqrt(cvx * cvx + cvy * cvy));

        //facing vector * thrust
        double outx = cos(cangle) * cthrust;
        double outy = sin(cangle) * cthrust;
        
        cvx += outx; //changing current speed vector
        cvy += outy;

        cx = round(cx + cvx); //change position
        cy = round(cy + cvy);

        cvx = (int)(cvx * 0.85); //friction
        cvy = (int)(cvy * 0.85);

        been_inside |= point_in_circle(cx, cy, check_x[nc[0]], check_y[nc[0]], check_radius);
        //cerr << "Step: " << csteps << " x: " << cx << " y: " << cy << " vx: " <<  cvx << " vy: " << cvy << " angle: " << (cangle * 180 / PI) << " been_in: " << been_inside << "\n";
        csteps++;
    }

    return been_inside;
}

void racing_drone(){
    /*
    if (abs(nc_deg_angle[0]) > 90 || nc_dist[0] / v[0] <= 1){ //slow down if needed
        thrust[0] = 0;
    }
    */

    tarx[0] = check_x[nc[0]] - vx[0];
    tary[0] = check_y[nc[0]] - vy[0];

    bool nnc_steer = check_nnc_steer();
    if(nnc_steer/* || nc_dist[0] < 2000*/){
        tarx[0] = next_check_x(nc[0]) - vx[0];
        tary[0] = next_check_y(nc[0]) - vy[0];
    }

    double tar_angle = vector_angle(tarx[0] - x[0], tary[0] - y[0]);
    double tar_angle_delta = angle_diff(angle[0], tar_angle);
    thrust[0] = curving_speed(tar_angle_delta, v[0]);

    double tar_check_delta = angle_diff(vector_angle(check_x[nc[0]] - x[0], check_y[nc[0]] - y[0]), tar_angle);

    printErr(tar_angle);

    if(should_shield(0)){
        cout << (int)(tarx[0]) << " " << (int)(tary[0]) << " " << "SHIELD" << " shield racing\n";
    }else if((/*_time == 0 || */(!nnc_steer && thrust[0] == 100 && abs(tar_angle_delta * 180 / PI) <= 5 && abs(tar_check_delta * 180 / PI) <= 5 && nc_dist[0] >= 5000)) && !used_boost){
        cout << (int)(tarx[0]) << " " << (int)(tary[0]) << " " << "BOOST" << " boost racing\n";
        used_boost = true;
    }else{
        cout << (int)(tarx[0]) << " " << (int)(tary[0]) << " " << thrust[0] << " racing " << nnc_steer << "\n";
    }
}

void fix_wanted_in_check(double tetha){
    bool bad = true;
    while(bad){
        bad = false;
        for(int i = 0; i < check_count; ++i){
            bool inc = point_in_circle(colWantedx, colWantedy, check_x[i], check_y[i], check_radius);
            if(inc){
                bad = true;
                wanted_dist += 1000;
                colWantedx = check_x[check_count - 1] + cos(tetha) * wanted_dist;
                colWantedy = check_y[check_count - 1] + sin(tetha) * wanted_dist;
            }
        }
    }
}

double calc_wanted(){
    double vxsll = check_x[check_count - 1] - check_x[check_count - 2];
    double vysll = check_y[check_count - 1] - check_y[check_count - 2];
    double sllangle = vector_angle(vxsll, vysll);
    double vxl0 = check_x[0] - check_x[check_count - 1];
    double vyl0 = check_y[0] - check_y[check_count - 1];
    double l0angle = vector_angle(vxl0, vyl0);
    double turn_angle = angle_diff(sllangle, l0angle);
    int sgn = sign(turn_angle);
    double tetha = sllangle + turn_angle / 2 + -1 * sgn * PI / 2;

    colWantedx = check_x[check_count - 1] + cos(tetha) * wanted_dist;
    colWantedy = check_y[check_count - 1] + sin(tetha) * wanted_dist;

    fix_wanted_in_check(tetha);

    return tetha;
}

void collider_going_wanted(){
    tarx[1] = colWantedx;
    tary[1] = colWantedy;
    thrust[1] = 100;

    double dtw = point_dist(x[1], y[1], colWantedx, colWantedy);
    double dist_to_stop = max(v[1], 100.0) * (pow(0.85, 21) - 1) / (0.85 - 1); //if i assume time to stop is 20
    if(dtw <= dist_to_stop){ //should use better metric to know when to start slowing down
        tarx[1] = check_x[check_count - 1];
        tary[1] = check_y[check_count - 1];
        thrust[1] = 0;

        colliderState = SLOWING_WANTED;
    } //i am rotating and hoping i end up near the wanted point
}

void collider_slowing_wanted(){
    tarx[1] = check_x[check_count - 1];
    tary[1] = check_y[check_count - 1];
    thrust[1] = 0;
    
    if(v[1] <= 20){
        colliderState = WAITING;
    }
}

void collider_waiting(int leading){
    tarx[1] = check_x[check_count - 1];
    tary[1] = check_y[check_count - 1];
    thrust[1] = 0;

    //leading distance to target
    double ldtt = point_dist(opp_x[leading], opp_y[leading], check_x[check_count - 1], check_y[check_count - 1]);

    double ldtoc = opp_to_check(leading, check_count - 1); //him to check
    double dtc = point_dist(x[1], y[1], tarx[1], tary[1]); //me to check
    double const my_speedup_dist = 2500;

    if(ldtoc <= dtc + my_speedup_dist){ //atrocity of a check
        thrust[1] = 100;
        colliderState = GOING_TARGET;
        col_operation_start = _time;
    }
}

void collider_going_target(int leading){
    static const int operation_time = 10;

    if(_time >= col_operation_start + operation_time){
        tarx[1] = colWantedx;
        tary[1] = colWantedy;
        thrust[1] = 100;
        colliderState = GOING_WANTED;
        return;
    }

    tarx[1] = check_x[check_count - 1];
    tary[1] = check_y[check_count - 1];
    thrust[1] = 100;

    //distance to leading
    double dtl = point_dist(x[1], y[1], opp_x[leading], opp_y[leading]);

    if(dtl <= 2000){ //okayish check
        tarx[1] = opp_x[leading] + opp_vx[leading];
        tary[1] = opp_y[leading] + opp_vy[leading];
        if(dtl <= 1000){
            tarx[1] = opp_x[leading];
            tary[1] = opp_y[leading];
        }
    }
}


void collider_drone(){
    int leading = leading_opponent_drone() ? 1 : 0;

    cerr << "Drone 1: " << opp_x[0] << " " << opp_y[0] << " " << opp_laps[0] << " " << opp_nc[0] << " " << opp_to_nc(0) << "\n"; 
    cerr << "Drone 2: " << opp_x[1] << " " << opp_y[1] << " " << opp_laps[1] << " " << opp_nc[1] << " " << opp_to_nc(1) << "\n"; 
    cerr << "Leading: " << leading + 1 << "\n";

    if(_time <= 2){
        cout << (int)colWantedx << " " << (int)colWantedy << " 0 colli\n";
        return;
    }

    switch (colliderState)
    {
        case GOING_WANTED:
            collider_going_wanted();
            break;
        case SLOWING_WANTED:
            collider_slowing_wanted();
            break;
        case WAITING:
            collider_waiting(leading);
            break;
        case GOING_TARGET:
            collider_going_target(leading);
            break;
    }

    if(should_shield(1)){
        cout << (int)tarx[1] << " " << (int)tary[1] << " SHIELD SHIELD colli\n";
    } else {
        cout << (int)tarx[1] << " " << (int)tary[1] << " " << thrust[1] << " colli\n";
    }
}

void solve(){
    turn_input();

    racing_drone(); // 0
    collider_drone(); // 1

    _time++;
}

void test_line(){
    bool a = line_inter_circle(0, 0, 11330 - 10732, 7547 - 7519, 14102 - 11330, 7751 - 7547, 600);
    a = line_inter_circle(0, 0, 598, 28, 2772, 204, 600);
    a = line_inter_circle(0, 0, 439, -377, 1471, -2474, 600);
    cout << a << "\n";
}

void test_angle_diff(){
    double res = angle_diff(210 * PI / 180, 262.9 * PI / 180);
    cout << res << "\n";
}

void test_calc_wanted(){
    check_count = 3;
    check_x[1] = 0;
    check_y[1] = 0;
    check_x[2] = 0;
    check_y[2] = 1;
    check_x[0] = -2;
    check_y[0] = 0.5;
    //wanted_dist = 2;
    double angl = calc_wanted();
    cout << "Angle: " << angl << " x: " << colWantedx << " y: " << colWantedy << "\n";
}

int main()
{
    //test_calc_wanted();
    //return 0;

    cin >> laps; cin.ignore();
    cin >> check_count; cin.ignore();
    for(int i = 0; i < check_count; ++i){
        cin >> check_x[i] >> check_y[i]; cin.ignore();
    }
    calc_wanted();

    while (1) {
        solve();
    }
    return 0;
}
