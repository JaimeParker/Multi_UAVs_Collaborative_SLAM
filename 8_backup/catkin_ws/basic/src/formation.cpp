//
// Created by hazyparker on 2022/1/20.
// formation transform test
// https://leetcode-cn.com/problems/maximum-compatibility-score-sum/solution/c-km-suan-fa-er-fen-tu-dai-quan-zui-da-p-ztmd/


#include <tuple>
#include <cmath>

using namespace std;

typedef tuple<double, double, double> Coordinate;

struct Offset{
    double x = 0;
    double y = 0;
    double z = 0;

    double x_des = 0;
    double y_des = 0;
    double z_des = 0;
};

double get_distance(struct Offset a, struct Offset b){
    return sqrt(pow(a.x - b.x, 2) + pow(a.y -b.y, 2) + pow(a.z - b.z, 2));
}

int main(){
    // define and init data
    Coordinate coordinate0, coordinate1, coordinate2, coordinate3, coordinate4, coordinate5;
    // x, y, z coordinates in order, respectively
    get<0>(coordinate0) = 0;
    get<1>(coordinate0) = 0;
    get<2>(coordinate0) = 8;

    get<0>(coordinate1) = 2;
    get<1>(coordinate1) = 0;
    get<2>(coordinate1) = 8;

    get<0>(coordinate2) = -2;
    get<1>(coordinate2) = 0;
    get<2>(coordinate2) = 8;

    get<0>(coordinate3) = 0;
    get<1>(coordinate3) = 0;
    get<2>(coordinate3) = 6;

    get<0>(coordinate4) = 0;
    get<1>(coordinate4) = 0;
    get<2>(coordinate4) = 4;

    get<0>(coordinate5) = 0;
    get<1>(coordinate5) = 0;
    get<2>(coordinate5) = 2;

    // init offset variables
    Offset offset1{}, offset2{}, offset3{}, offset4{}, offset5{};
    offset1.x = get<0>(coordinate1) - get<0>(coordinate0);
    offset1.y = get<1>(coordinate1) - get<1>(coordinate0);
    offset1.z = get<2>(coordinate1) - get<2>(coordinate0);

    offset2.x = get<0>(coordinate2) - get<0>(coordinate0);
    offset2.y = get<1>(coordinate2) - get<1>(coordinate0);
    offset2.z = get<2>(coordinate2) - get<2>(coordinate0);

    offset3.x = get<0>(coordinate3) - get<0>(coordinate0);
    offset3.y = get<1>(coordinate3) - get<1>(coordinate0);
    offset3.z = get<2>(coordinate3) - get<2>(coordinate0);

    offset4.x = get<0>(coordinate4) - get<0>(coordinate0);
    offset4.y = get<1>(coordinate4) - get<1>(coordinate0);
    offset4.z = get<2>(coordinate4) - get<2>(coordinate0);

    offset5.x = get<0>(coordinate5) - get<0>(coordinate0);
    offset5.y = get<1>(coordinate5) - get<1>(coordinate0);
    offset5.z = get<2>(coordinate5) - get<2>(coordinate0);

    // init formation offset
    Offset form_off1{}, form_off2{}, form_off3{}, form_off4{}, form_off5{};
    form_off1.z = -4;

    form_off2.x = 2;
    form_off2.z = -2;

    form_off3.x = -2;
    form_off3.z = -2;

    form_off4.y = 2;
    form_off4.z = -2;

    form_off5.y = -2;
    form_off5.z = -2;




    return 0;
}
