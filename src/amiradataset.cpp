#include "amiradataset.hh"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <stdio.h>

using namespace std;

//--------------------------------------------------------------------------//
namespace RS
{
    //--------------------------------------------------------------------------//
    const real DBL_MAX = __DBL_MAX__;
    real AmiraDataSet::ads_bbox[8] = {-DBL_MAX, DBL_MAX, -DBL_MAX, DBL_MAX,
                                      -DBL_MAX, DBL_MAX, -DBL_MAX, DBL_MAX};

    //--------------------------------------------------------------------------//
    AmiraDataSet::AmiraDataSet(const std::string &filename)
        : Flow3D(&ads_bbox[0], "Amira")
    {
        m_filenames.clear();
        m_filenames.push_back(filename);
        ads_bbox[6] = 0.0;
        ads_bbox[7] = 0.0;
        init();
        Flow3D::init(&ads_bbox[0]);
    }

    //--------------------------------------------------------------------------//
    AmiraDataSet::AmiraDataSet(const std::vector<std::string> &filenames,
                               const Vec2r &timeInfo)
        : Flow3D(&ads_bbox[0], "Amira"), m_filenames(filenames)
    {
        ads_bbox[6] = timeInfo[0];
        ads_bbox[7] = timeInfo[1];
        init();
        Flow3D::init(&ads_bbox[0]);
    }

    //--------------------------------------------------------------------------//
    AmiraDataSet::AmiraDataSet(const AmiraDataSet &other)
        : Flow3D(other.mp_domain, other.m_name), m_filenames(other.m_filenames)
    {
        *ads_bbox = *other.ads_bbox;
        init();
        Flow3D::init(&ads_bbox[0]);
    }

    //--------------------------------------------------------------------------//
    AmiraDataSet &
    AmiraDataSet::operator=(const AmiraDataSet &other)
    {
        m_filenames = other.m_filenames;
        *ads_bbox = *other.ads_bbox;
        init();
        Flow3D::init(&ads_bbox[0]);
        return *this;
    }

    //--------------------------------------------------------------------------//
    AmiraDataSet::~AmiraDataSet(void)
    {
        delete[] mp_gridData;
    }

    //--------------------------------------------------------------------------//
    Vec3r
    AmiraDataSet::v(real t, const Vec3r &pos) const
    {
        if (isSteady())
            t = m_steady_time;
        if (isPeriodic())
            t = clampTime(t);
        Vec3r v(0.0, 0.0, 0.0);
        Vec4r tempPos(pos[0], pos[1], pos[2], t);
        Vec4rField tempField(m_vec4Grid);
        tempField.value(v, tempPos); // ASDF here we get a warning
        return v;
    }

    //--------------------------------------------------------------------------//
    void
    AmiraDataSet::init(const real *bbox)
    {
        Flow3D::init(bbox);
    }

    //--------------------------------------------------------------------------//
    void
    AmiraDataSet::init(void)
    {
        // time component is defined by the number of files we have
        m_dimT = static_cast<int>(m_filenames.size());

        // load the different data sets
        // the single slices are loaded as 3D data sets
        // and then stored after each other as '4D' data
        int IdxGrid = 0; // position in 4D array
        for (int fileId = 0; fileId < m_dimT; fileId++)
        {
            // load the data from file
            cout << "\rLoad file " << fileId + 1 << "/" << m_dimT << flush;
            float *data;
            if (!loadFile(m_filenames[fileId], &data, fileId == (m_dimT - 1)))
                return;

            // first run specifies the grid size
            if (!mp_gridData)
                mp_gridData = new Vec3r[m_dimX * m_dimY * m_dimZ * m_dimT];

            // convert the float data into a grid
            int Idx = 0;
            for (int k = 0; k < m_dimZ; k++)
                for (int j = 0; j < m_dimY; j++)
                    for (int i = 0; i < m_dimX; i++)
                    {
                        //********
                        float x = data[Idx * m_components + 0];
                        float y = data[Idx * m_components + 1];
                        float z = data[Idx * m_components + 2];
                        Vec3r gridVec(x, y, z);
                        mp_gridData[IdxGrid] = gridVec;
                        Idx++;
                        IdxGrid++;
                    }
            delete[] data;
        }

        //------- HACK BEGIN ---------
        // handle data sets with only one timeslice
        if (1 == m_dimT)
        {
            Vec3r *gridData = new Vec3r[m_dimX * m_dimY * m_dimZ * 2];
            int Idx0 = 0;
            int Idx1 = 0;
            for (int l = 0; l < 2; l++)
            {
                Idx0 = 0;
                for (int k = 0; k < m_dimZ; k++)
                    for (int j = 0; j < m_dimY; j++)
                        for (int i = 0; i < m_dimX; i++)
                        {
                            gridData[Idx1] = mp_gridData[Idx0];
                            Idx0++;
                            Idx1++;
                        }
            }
            delete[] mp_gridData;
            mp_gridData = gridData;
            m_dimT = 2;

            ads_bbox[7] = 1.0;
        }

        //-------- HACK END ----------

        real xmin = ads_bbox[0];
        real xmax = ads_bbox[1];
        real ymin = ads_bbox[2];
        real ymax = ads_bbox[3];
        real zmin = ads_bbox[4];
        real zmax = ads_bbox[5];
        real tmin = ads_bbox[6];
        real tmax = ads_bbox[7];

        Vec4rGrid tempGrid(mp_gridData,
                           VC::mvfields::d4::msize_t(m_dimX, m_dimY, m_dimZ, m_dimT));
        tempGrid.set_origin(Vec4rGrid::coord_t(xmin, ymin, zmin, tmin));
        tempGrid.set_extent(
            Vec4rGrid::coord_t(xmax - xmin, ymax - ymin, zmax - zmin, tmax - tmin));
        m_vec4Grid = tempGrid;
    }

    // ------------------------------------------------------------------------ //
    /** Find a string in the given buffer and return a pointer to the contents
     * directly behind the SearchString. If not found, return the buffer. A
     * subsequent sscanf() will fail then, but at least we return a decent pointer. */
    const char *
    AmiraDataSet::findAndJump(const char *buffer, const char *SearchString)
    {
        const char *FoundLoc = strstr(buffer, SearchString);
        if (FoundLoc)
            return FoundLoc + strlen(SearchString);
        return buffer;
    }

    //--------------------------------------------------------------------------//
    bool
    AmiraDataSet::loadFile(const std::string &filename,
                           float **p_data,
                           bool verbose,
                           bool print_header)
    {
        ifstream content(filename, ios::in | ios::binary);
        char *data = new char[2048];
        content.read(data, sizeof(char) * 2048);
        string header = string(data, 2048);
        delete[] data;
        //---
        if (print_header)
        {
            cout << "------------------------\n";
            cout << header << endl;
        }

        //---
        string substring = "# AmiraMesh BINARY-LITTLE-ENDIAN 2.1";
        if (header.find(substring) == string::npos)
        {
            cout << "Not a proper AmiraMesh file." << endl;
            content.close();
            return false;
        }

        //---
        // Find the Lattice definition, i.e., the dimensions of the uniform grid
        substring = "define Lattice ";
        size_t string_pos = header.find(substring);
        if (string_pos == string::npos)
        {
            cout << "Could not find lattice information." << endl;
            content.close();
            return false;
        }
        string_pos += substring.length();
        content.seekg(string_pos, ios::beg);
        string token;
        // clang-format off
        getline(content, token, ' ');  m_dimX = atoi(&token[0]);
        getline(content, token, ' ');  m_dimY = atoi(&token[0]);
        getline(content, token);       m_dimZ = atoi(&token[0]);
        // clang-format on
        if (verbose)
        {
            cout << "\nExtracted data:\n";
            cout << "Lattice is: " << m_dimX << " " << m_dimY << " " << m_dimZ << endl;
        }

        //---
        // Find the bounding box information
        substring = "BoundingBox ";
        string_pos = header.find(substring);
        if (string_pos == string::npos)
        {
            cout << "Could not bounding box." << endl;
            content.close();
            return false;
        }
        string_pos += substring.length();
        content.seekg(string_pos, ios::beg);
        float xmin{1}, ymin{1}, zmin{1}, xmax{-1}, ymax{-1}, zmax{-1};
        // clang-format off
        getline(content, token, ' '); xmin = atof(&token[0]);
        getline(content, token, ' '); xmax = atof(&token[0]);
        getline(content, token, ' '); ymin = atof(&token[0]);
        getline(content, token, ' '); ymax = atof(&token[0]);
        getline(content, token, ' '); zmin = atof(&token[0]);
        getline(content, token);      zmax = atof(&token[0]);
        // clang-format on

        if (verbose)
        {
            cout << "Bounding box in x-direction: [" << xmin << ", " << xmax << "]\n";
            cout << "Bounding box in y-direction: [" << ymin << ", " << ymax << "]\n";
            cout << "Bounding box in z-direction: [" << zmin << ", " << zmax << "]\n";
        }

        AmiraDataSet::ads_bbox[0] = xmin;
        AmiraDataSet::ads_bbox[1] = xmax;
        AmiraDataSet::ads_bbox[2] = ymin;
        AmiraDataSet::ads_bbox[3] = ymax;
        AmiraDataSet::ads_bbox[4] = zmin;
        AmiraDataSet::ads_bbox[5] = zmax;

        //---
        // check if we have uniform data
        substring = "CoordType \"uniform\"";
        string_pos = header.find(substring);
        if (string_pos == string::npos)
        {
            cout << "Could not find 'CoordType \"uniform\"'." << endl;
            content.close();
            return false;
        }

        //---
        // type of the field: scalar, vector
        int num_components = {0};
        substring = "Lattice { float Data }";
        string_pos = header.find(substring);
        if (string_pos != string::npos)
            num_components = 1;
        //
        substring = "Lattice { float[";
        string_pos = header.find(substring);
        if (string_pos != string::npos)
        {
            string_pos += substring.length();
            content.seekg(string_pos, ios::beg);
            getline(content, token, ']');
            num_components = atoi(&token[0]);
        }
        if (verbose)
            cout << "Number of components: " << num_components << endl;
        m_components = num_components;

        //---
        // sanity check
        // clang-format off
  if (m_dimX <= 0 || m_dimY <= 0 || m_dimZ <= 0 ||
      xmin > xmax || ymin > ymax || zmin > zmax ||
      num_components <= 0) {
    cout << "Could not read amira file" << endl;
    content.close();
    return false;
  }
        // clang-format on

        //---
        // find the beginning of the data section
        substring = "# Data section follows";
        string_pos = header.find(substring);
        if (string_pos == string::npos)
        {
            cout << "Could not find '# Data section follows'." << endl;
            content.close();
            return false;
        }
        content.seekg(string_pos, ios::beg);
        // consume the next two lines which area "# Data section follows" and "@1"
        getline(content, token);
        getline(content, token);
        // read the data
        // - how much to read
        const size_t num_to_read = m_dimX * m_dimY * m_dimZ * num_components;
        // - prepare memory
        *p_data = new float[num_to_read];
        // - do it
        content.read((char *)*p_data, sizeof(float) * num_to_read);
        content.close();
        if (verbose)
            cout << "Succesfully loaded amira file\n"
                 << endl;
        return true;
    }
    //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//