#pragma once
#include "flow.hh"
#include "vclibs/mvfields/grid_d2.hh"
#include "vclibs/mvfields/grid_d3.hh"
#include "vclibs/mvfields/grid_d4.hh"

//--------------------------------------------------------------------------//
namespace RS
{
    //--------------------------------------------------------------------------//
    class AmiraDataSet : public Flow3D
    {
    public:
        //--------------------------------------------------------------------------//
        AmiraDataSet(const std::string &filename);
        AmiraDataSet(const std::vector<std::string> &filenames,
                     const Vec2r &timeInfo);
        AmiraDataSet(const AmiraDataSet &other);
        AmiraDataSet &operator=(const AmiraDataSet &other);
        //--------------------------------------------------------------------------//
        virtual ~AmiraDataSet(void);
        //--------------------------------------------------------------------------//
        /// compute flow
        virtual Vec3r v(real t, const Vec3r &pos) const override;
        //--------------------------------------------------------------------------//
    protected:
        //--------------------------------------------------------------------------//
        virtual void init(const real *bbox) override;
        //--------------------------------------------------------------------------//
        static real ads_bbox[8];
        //--------------------------------------------------------------------------//
        // methods to initialise and load the data
        virtual void init(void);
        //--------------------------------------------------------------------------//
        static const char *findAndJump(const char *buffer, const char *SearchString);
        //--------------------------------------------------------------------------//
        bool loadFile(const std::string &filename,
                      float **pData,
                      bool verbose = false,
                      bool print_header = false);
        //--------------------------------------------------------------------------//
        // members to handle the loaded data
        std::vector<std::string> m_filenames; // comes as input
        //--------------------------------------------------------------------------//
        int m_dimX = 0;
        int m_dimY = 0;
        int m_dimZ = 0;
        int m_dimT = 0;
        int m_components = 0;
        Vec3r *mp_gridData = nullptr;
        //--------------------------------------------------------------------------//
        typedef VC::mvfields::d4::grid<real, Vec3r> Vec4rGrid;
        typedef VC::mvfields::d4::kernel::linear_interpolatory_c0<Vec4rGrid>
            Vec4rKernel;
        typedef VC::mvfields::d4::evaluator<Vec4rKernel> Vec4rField;
        //--------------------------------------------------------------------------//
        Vec4rGrid m_vec4Grid; // Contains sampled vector grids, sorted to by tau value
        //--------------------------------------------------------------------------//
    };
    //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//