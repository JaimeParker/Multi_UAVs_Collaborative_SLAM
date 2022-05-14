#include <iostream>
#include "include/Data.h"


using namespace std;

int main() {
    string path1 = "../image/img1.jpg";
    string path2 = "../image/img2.jpg";
    string path3 = "../image/img3.jpg";
    string path4 = "../image/img4.jpg";
    Data data(path1, path2, path3, path4);

    return 0;
}
