#include "globals.hh"

using namespace RS;

real Globals::EPS   = std::numeric_limits<double>::epsilon();
real Globals::ZERO  = 1000 * Globals::EPS;
real Globals::SMALL = 10000000 * Globals::EPS;

real Globals::SEARCHPREC = 0.001;

real Globals::RAYBACKOFFSET_REFINEMENT     = 0.015;
real Globals::RAYFOREOFFSET_SHADOWS        = 0.005;

real Globals::NORMAL_SEARCHDIS  = 0.005; // total HL length is double the value (dis in both directions)
size_t Globals::NORMAL_MAXSTEPS = 3;

real Globals::NEIGHBOR_SPACEANGLE   = 85.0 / 180.0 * M_PI;
real Globals::NEIGHBOR_DIFT0_PERLU  = 60.0; // DG: 60 | SC: 20
real Globals::NEIGHBOR_DIFTAU_PERLU = 60.0; // DG: 60 | SC: 20

real Globals::SPACEEQUAL    = 0.00005;
real Globals::T0EQUAL       = 0.00005;
real Globals::TAUEQUAL      = 0.00005;
real Globals::TAUMIN        = 0.001;
real Globals::DETMIN        = 0.000001;
real Globals::RECPOINTEQUAL = 0.00005;