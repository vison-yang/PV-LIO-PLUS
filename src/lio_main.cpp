/** @file lio_main.cpp @brief PV-LIO-PLUS node entry point. */
#include "mapping.hpp"

int main(int argc, char** argv) {
    return pv_lio_plus::RunMappingNode(argc, argv);
}
