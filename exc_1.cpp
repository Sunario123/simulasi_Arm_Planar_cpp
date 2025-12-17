#include <bits/stdc++.h>

using namespace std;

double l1, l2;
double theta1, theta2;
double thetaDot1, thetaDot2;

double hitungKiriAtas(){
    return ((-1)*(l1)*sin(theta1))-(l2*sin(theta1+theta2));
}

double hitungKiriBawah(){
    return ((l1)*cos(theta1))+(l2*cos(theta1+theta2));
}

double hitungKananAtas(){
    return (-1)*l2*sin(theta1+theta2);
}

double hitungKananBawah(){
    return l2*cos(theta1+theta2);
}

double deg2rad(double deg) {
    return deg * M_PI / 180.0; // Convert dari derajat ke radian
}

void hitungKecepatanLinear(){
    cout<<"l1: "; cin>>l1;
    cout<<"l2: "; cin>>l2;
    cout<<"theta1 (derajat): "; cin>>theta1;
    cout<<"theta2 (derajat): "; cin>>theta2;
    cout<<"thetaDot1 (rad/s): "; cin>>thetaDot1; // Dalam radian / sekon
    cout<<"thetaDot2 (rad/s): "; cin>>thetaDot2; // Dalam radian / sekon
    
    theta1 = deg2rad(theta1);
    theta2 = deg2rad(theta2);
    
    double kecepatanLinearX = (hitungKiriAtas()*thetaDot1)+(hitungKananAtas()*thetaDot2);
    double kecepatanLinearY = (hitungKiriBawah()*thetaDot1)+(hitungKananBawah()*thetaDot2);
    
    cout << "xdot = " << kecepatanLinearX << " m/s" << endl;
    cout << "ydot = " << kecepatanLinearY << " m/s" << endl;
}

void hitungKecepatanSudut(){

    double kecepatanLinearX;
    double kecepatanLinearY;

    cout<<"l1: "; cin>>l1;
    cout<<"l2: "; cin>>l2;
    cout<<"theta1 (derajat): "; cin>>theta1;
    cout<<"theta2 (derajat): "; cin>>theta2;
    cout<<"xDot: "; cin>>kecepatanLinearX; // Dalam m/s
    cout<<"yDot: "; cin>>kecepatanLinearY; // Dalam m/s
    
    theta1 = deg2rad(theta1);
    theta2 = deg2rad(theta2);

    double detJ = (hitungKiriAtas() * hitungKananBawah())
                - (hitungKananAtas() * hitungKiriBawah());

    if (fabs(detJ) < 1e-6) {
        cout << "Jacobian singular! Tidak bisa di-inverse.\n";
        return;
    }

    thetaDot1 = ((hitungKananBawah()*kecepatanLinearX)
                -(hitungKananAtas()*kecepatanLinearY)) / detJ;

    thetaDot2 = ((-hitungKiriBawah()*kecepatanLinearX)
                +(hitungKiriAtas()*kecepatanLinearY)) / detJ;

    
    cout << "thetaDot1 = " << thetaDot1 << " rad/s" << endl;
    cout << "thetaDot2 = " << thetaDot2 << " rad/s" << endl;

}

void forwardKinematics(double &x, double &y){
    x = l1 * cos(theta1) + l2 * cos(theta1 + theta2);
    y = l1 * sin(theta1) + l2 * sin(theta1 + theta2);
}

void kontrolResolvedRate(){

    double xd, yd;
    double K;
    double dt;
    double eps = 1e-3;

    cout<<"l1: "; cin>>l1;
    cout<<"l2: "; cin>>l2;
    cout<<"theta1 (deg): "; cin>>theta1;
    cout<<"theta2 (deg): "; cin>>theta2;
    cout<<"xd: "; cin>>xd;
    cout<<"yd: "; cin>>yd;
    cout<<"Gain K: "; cin>>K;
    cout<<"dt: "; cin>>dt;

    theta1 = deg2rad(theta1);
    theta2 = deg2rad(theta2);

    int iter = 0;

    while(true){

        // forward kinematics
        double x, y;
        forwardKinematics(x, y);

        // error posisi
        double ex = xd - x;
        double ey = yd - y;
        double error = sqrt(ex*ex + ey*ey);

        //Jika eror mendekati 0 maka itu dianggap sudah valid
        if(error < eps){
            cout<<"Target tercapai\n";
            break;
        }

        // kecepatan linear end-effector
        double xDot = K * ex;
        double yDot = K * ey;

        // Determinan Jacobian
        double detJ = (hitungKiriAtas() * hitungKananBawah())
                    - (hitungKananAtas() * hitungKiriBawah());

        //Jika determinan 0 maka tidak bisa di invers 
        if(fabs(detJ) < 1e-6){
            cout<<"Jacobian singular! Stop.\n";
            break;
        }

        // inverse Jacobian * xDot
        thetaDot1 = ((hitungKananBawah()*xDot)
                    - (hitungKananAtas()*yDot)) / detJ;

        thetaDot2 = ((-hitungKiriBawah()*xDot)
                    + (hitungKiriAtas()*yDot)) / detJ;

        // update sudut
        theta1 += thetaDot1 * dt;
        theta2 += thetaDot2 * dt;

        // untuk output seperti di stm32cube ide
        if(iter % 50 == 0){
            cout<<"Iter "<<iter<<" | x="<<x<<" y="<<y<<" | error="<<error<<endl;
        }

        iter++;
    }

    cout<<"theta1 akhir = "<<theta1<<" rad\n";
    cout<<"theta2 akhir = "<<theta2<<" rad\n";
}

int main(){
    while(true){

        system("clear");

        int option;
        cout<<"1. hitung kecepatan linear"<<endl;
        cout<<"2. hitung kecepatan sudut"<<endl;
        cout<<"3. Kontrol Resolved Rate"<<endl;
        cout<<"Select option: ";

        cin>>option;

        switch (option)
        {
        case 1:
            hitungKecepatanLinear();
            break;
        
        case 2:
            hitungKecepatanSudut();
            break;

        case 3: 
            kontrolResolvedRate();
            break;

        default:
            cout<<"Tidak dalam opsi!!"<<endl;
            break;
        }

        char check;
        cout<<"Again? (y/n)"; cin>>check;
        if(check == 'y') continue;
        else break;
    }
}