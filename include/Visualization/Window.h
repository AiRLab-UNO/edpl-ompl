/* Window stub — no Qt dependency */
#ifndef WINDOW_H
#define WINDOW_H
#include <string>
using namespace std;
class MyWindow {
public:
    MyWindow() {}
    void resize(int, int) {}
    void showMaximized() {}
    void resetCamera() {}
    struct Size { int width=800, height=600; };
    Size sizeHint() const { return Size(); }
};
#endif
