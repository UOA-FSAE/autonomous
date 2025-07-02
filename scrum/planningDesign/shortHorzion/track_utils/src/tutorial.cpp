// #include <utility>

// #include <iostream>

// // lvalue = rvalue
// // at addr 10
// int a = 1;

// // ptr to lvalue
// // at addr 5
// int *b = &a;


// // ptr to ptr to lvalue
// // at addr 2
// int **bp = &b;

// // reference to an lvalue
// // at addr 10
// int &c = a;

// // at addr 55
// int d = a;

// // const lvalue
// int const e = 1;

// // const reference to a const lvalue
// int const &f = e;

// // const refernce to a lvalue
// int const &g = a;

// // reference to and rvalue
// int &&h = 1; 



// void assign(int &&other) {
//     // addr 10
//     a = 5;
// }


// class Thing {
//     public:
//     int a;
//     int b = 10;
//     bool c = false;

//     Thing() : a(1)  {}
// };


// Thing&& getThing() {
//     return Thing();
// }

// // lvalue = lvalue
// Thing b = getThing();






// main() {
//     // addr 10
//     a = 1;

//     // addr 777
//     int the_other = 1000;

//     assign(std::move(the_other));



// }




// int counter() {
//     static int count = 0; 
//     return count++;
// }




// /*
// >> main.exe
// 0
// 1
// 2
// 3
// 4
// 5
// 6
// 7
// 8
// 9
// */



// class Animal {
//     static bool fur; 
//     Animal() {

//     }

//     void disableFur() {
//         fur = false;
//     }

//     void setFur(bool other_fur) {
//         Animal::fur = other_fur;
//     }
// };

