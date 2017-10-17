

#include "VideoFile.h"
#include "Slam.h"
#include "View.h"


int main(int argc, char** argv) {

    ww::VideoSource* vs = new ww::VideoFile(argv[1]);
    ww::ViewContent* vc = new ww::Slam(vs);
    ww::View view(vc);
    view.run();

    return 0;

}
