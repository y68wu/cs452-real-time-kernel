#include <stdio.h>

#include "../tc2_ui_path_geometry.h"

int main(void) {
        if (Tc2UiPathGeometryValidate() < 0) {
                fprintf(stderr,
                        "tc2_ui_path_geometry_test: invalid path catalog\n");
                return 1;
        }
        printf("tc2_ui_path_geometry_test: PASS "
               "(51 matrix bends use ordered rail cells)\n");
        return 0;
}
