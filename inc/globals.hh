#pragma once

#include "types.hh"

//--------------------------------------------------------------------------//
namespace RS
{
  //--------------------------------------------------------------------------//
  class Globals
  {
    //--------------------------------------------------------------------------//
  private:
    Globals() {}
    //--------------------------------------------------------------------------//
  public:
    //--------------------------------------------------------------------------//
    /* General variables */
    static real EPS;   // smallest possible value difference
    static real ZERO;  // quasi-zero (very small)
    static real SMALL; // small value, greater than ZERO
    //--------------------------------------------------------------------------//
    /* For control over hyperlines */
    static real SEARCHPREC; // threshold for recursive search in spatial distance
    //--------------------------------------------------------------------------//
    /* Offsets for searching */
    static real RAYBACKOFFSET_REFINEMENT; // ray offset for searching before estimated intersection
    static real RAYFOREOFFSET_SHADOWS;    // ray offset for skipping a first part of the ray to avoid finding the same point when checking shadows
    //--------------------------------------------------------------------------//
    /* Settings for normal calculation */
    static real NORMAL_SEARCHDIS;  // for normal estimation: defines how far the hyperlines are away from the RP
    static size_t NORMAL_MAXSTEPS; // maximum number of retrys with lower distance
    //--------------------------------------------------------------------------//
    /* Estimating neighboring recirculation points */
    static real NEIGHBOR_SPACEANGLE;   // max angle between 3D points so that they are considered as neighboring
    static real NEIGHBOR_DIFRAYPOS;    // max difference of ray positions so that they are considered as neighboring
    static real NEIGHBOR_DIFT0_PERLU;  // max t0 difference per length unit between points so that they are considered as neighboring
    static real NEIGHBOR_DIFTAU_PERLU; // max tau difference per length unit between points so that they are considered as neighboring
    //--------------------------------------------------------------------------//
    /* Estimating same recirculation points */
    static real SPACEEQUAL;    // max distance so that two 3D points are considered as the same point
    static real T0EQUAL;       // max t0 difference so that two 3D points are considered as the same point
    static real TAUEQUAL;      // max tau difference so that two 3D points are considered as the same point
    static real TAUMIN;        // minimal tau value; must not be 0
    static real DETMIN;        // minimal value for determinant
    static real RECPOINTEQUAL; // minimum distance
    //--------------------------------------------------------------------------//
  };
  //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//