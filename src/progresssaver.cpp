#include "progresssaver.hh"

using namespace std;

//--------------------------------------------------------------------------//
namespace RS
{
    //--------------------------------------------------------------------------//
    ProgressSaver::ProgressSaver(const string &save_dir, size_t cam_width, size_t cam_height)
        : start_index{0},
          saved{},
          waiting{},
          complete_rewrite{false},
          next_save_index{0},
          count_waiting_positives{0},
          file_start{save_dir + "/progress_start.txt"},
          file_vec{save_dir + "/progress_points.txt"},
          width{cam_width},
          height{cam_height},
          index_map{nullptr}
    {
        index_map = new size_t *[width];
        for (size_t x = 0; x < width; ++x)
        {
            index_map[x] = new size_t[height];
            for (size_t y = 0; y < height; ++y)
                index_map[x][y] = numeric_limits<size_t>::max();
        }
    }

    //--------------------------------------------------------------------------//
    ProgressSaver::~ProgressSaver()
    {
        for (size_t x = 0; x < width; ++x)
            delete[] index_map[x];
        delete[] index_map;
    }

    //--------------------------------------------------------------------------//
    void ProgressSaver::update(const RSIntersection &data)
    {
        // There are two cases: Adding a new entry while building the normal model
        // or updating an existing one
        if (data.cam_index >= start_index) // case 1
        {
            waiting.push_back(data);
            if (waiting.size() > 1 && waiting[waiting.size() - 2].cam_index < waiting.back().cam_index)
                sort(waiting.begin(), waiting.end(), [](RSIntersection a, RSIntersection b)
                     { return a.cam_index > b.cam_index; });
            if (data.rp)
                ++count_waiting_positives;

            while (!waiting.empty() && waiting.back().cam_index == start_index)
            {
                RSIntersection &obj = waiting.back();
                size_t x = obj.cam_index % width, y = obj.cam_index / width;
                if (obj.rp)
                {
                    --count_waiting_positives;
                    index_map[x][y] = saved.size();
                    saved.push_back(obj);
                }
                waiting.pop_back();
                ++start_index;
            }
        }
        else // case 2
        {
            size_t index = getRSIPosition(data.cam_index);
            if (data.rp)
            {
                complete_rewrite = true;
                if (index < width * height) // existing entry (simple case)
                {
                    saved[index] = data;
                }
                else // new entry (complicated)
                {
                    size_t insert_pos = saved.size();
                    for (size_t cam_index = width * height - 1; cam_index > data.cam_index && insert_pos > 0; --cam_index)
                    {
                        size_t x = cam_index % width, y = cam_index / width;
                        size_t &id = index_map[x][y];
                        if (id < saved.size())
                        {
                            ++id;
                            --insert_pos;
                        }
                    }
                    size_t x = data.cam_index % width, y = data.cam_index / width;
                    saved.insert(saved.begin() + insert_pos, data);
                    index_map[x][y] = insert_pos;
                }
            }
            // theoretically there is also the case that an entry can be deleted,
            // but it is not needed in this project
        }
    }

    //--------------------------------------------------------------------------//
    void ProgressSaver::saveData()
    {
        // save vector
        ofstream file;
        size_t start = 0;
        if (complete_rewrite)
            file.open(file_vec);
        else
        {
            file.open(file_vec, ios_base::app);
            start = next_save_index;
        }
        if (file)
        {
            for (size_t i = start; i < saved.size(); ++i)
            {
                RSIntersection p = saved[i];
                file << p.cam_index << ' '
                     << p.hit.value() << ' '
                     << p.rp.value().t0 << ' '
                     << p.rp.value().tau << "\n";
            }
        }
        file.close();
        next_save_index = saved.size();
        complete_rewrite = false;

        // save start index
        file = ofstream(file_start);
        if (file)
            file << start_index;
        file.close();
    }

    //--------------------------------------------------------------------------//
    void ProgressSaver::loadData(const Camera &cam)
    {
        ifstream file{file_start};
        if (file)
            file >> start_index;
        file = ifstream(file_vec);
        if (file)
        {
            bool dropped_last_point = false; // for checking if there are problems in the loaded file
            while (!file.eof())
            {
                size_t cam_index;
                file >> cam_index;

                Ray ray = cam.ray(cam_index % width, cam_index / width);
                real intersection;
                file >> intersection;

                Vec3r pos = ray(intersection);
                real t0, tau;
                file >> t0;
                if (file.eof()) // edge case: if last output was interrupted and data are corrupted
                {               // does not recognize error while writing last element
                    dropped_last_point = true;
                    break;
                }
                file >> tau;

                RSIntersection rsi{cam_index, ray, {intersection}, {{pos, t0, tau}}};
                saved.push_back(rsi);
            }
            file.close();

            // drop last point and adjust start_index if needed
            if (!saved.empty() && saved.back().cam_index >= start_index) // check error again
            {
                if (!dropped_last_point) // drop last point for security reasons if there was
                    saved.pop_back();    // an interruption while writing last variable
                start_index = saved.back().cam_index;
            }
        }
        // set other variables
        next_save_index = saved.size();
        complete_rewrite = false;
        count_waiting_positives = 0;

        // init index_map
        size_t vec_pos = 0;
        for (size_t cam_index = 0; cam_index < width * height; ++cam_index)
        {
            size_t x = cam_index % width, y = cam_index / width;
            if (vec_pos < saved.size() && saved[vec_pos].cam_index == cam_index)
                index_map[x][y] = vec_pos++;
            else
                index_map[x][y] = numeric_limits<size_t>::max();
        }
    }
    //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//