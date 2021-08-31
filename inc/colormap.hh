#pragma once

#include <algorithm>
#include "texture.hh"
#include "types.hh"

//--------------------------------------------------------------------------//
namespace RS
{
    //--------------------------------------------------------------------------//
    /// Get the mapped color for a given value between 0 and 1
    color getColorInferno(double p);
    /// Get the mapped color for a given value between 0 and 1
    color getColorViridis(double p);
    //--------------------------------------------------------------------------//
    /// Create a texture of the color mapping with defined width, heigth and border size
    Texture createColorMapTextureInferno(size_t width, size_t height, size_t border_size);
    /// Create a texture of the color mapping with defined width, heigth and border size
    Texture createColorMapTextureViridis(size_t width, size_t height, size_t border_size);
    //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//