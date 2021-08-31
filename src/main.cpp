#include <filesystem>
#include <stdio.h>

#include "refraytracer.hh"
#include "scenesetup.hh"
#include "shader.hh"
#include "timer.hh"

using namespace RS;
using namespace std;

//--------------------------------------------------------------------------//
void printSeparator(char c)
{
  for (size_t i = 0; i < 50; ++i)
    cout << c;
  cout << endl;
}

//--------------------------------------------------------------------------//
shared_ptr<Raytracer>
basicRaytracing(shared_ptr<Scene> scene,
                shared_ptr<Camera> cam,
                const string &save_dir)
{
  // setup directory
  filesystem::create_directories(save_dir);
  // create raytracer
  auto raytracer = make_shared<Raytracer>(cam, scene, save_dir + "/");
  // render space to see perspective
  raytracer->renderSpace();

  cout << "BASIC RAYTRACING (" + save_dir + ")" << endl;
  // start timer and execute rendering
  Timer timer{};
  timer.printStartTime();
  raytracer->render();
  timer.printTotalTime();
  // output of ratio and reset static timers
  TimerHandler::printRatio();
  TimerHandler::reset();

  printSeparator('=');
  return raytracer;
}

//--------------------------------------------------------------------------//
shared_ptr<RefinementRaytracer>
refiningRaytracing(const Raytracer *basic_rt,
                   size_t res_multiplier,
                   const string &save_dir,
                   bool do_postprocessing = true)
{
  // setup directory
  filesystem::create_directories(save_dir);
  // create raytracer building on previous results
  auto raytracer = make_shared<RefinementRaytracer>(basic_rt,
                                                    res_multiplier,
                                                    save_dir + "/");

  cout << "REFINEMENT (" + save_dir + ")" << endl;
  // start timer and execute rendering
  Timer timer{};
  timer.printStartTime();
  raytracer->render();
  timer.printTotalTime();
  // output of ratio and reset static timers
  TimerHandler::printRatio();
  TimerHandler::reset();

  printSeparator('-');

  cout << "POST PROCESSING (" + save_dir + ")" << endl;
  if (do_postprocessing)
  {
    // start timer and execute post processing
    timer = Timer();
    timer.printStartTime();
    raytracer->postProcessing();
    timer.printTotalTime();
    // output of ratio and reset static timers
    TimerHandler::printRatio();
    TimerHandler::reset();
  }
  else
    cout << "Skipped" << endl;

  printSeparator('=');
  return raytracer;
}

//--------------------------------------------------------------------------//
void shading(shared_ptr<Raytracer> raytracer,
             const string &save_dir,
             bool do_shadow_sharpening = true)
{
  // setup directory
  filesystem::create_directories(save_dir);

  Shader shader{raytracer, save_dir};

  cout << "SHADING (" + save_dir + ") - NORMALS BY NEIGHBORHOOD" << endl;
  // start timer and execute shading
  Timer timer{};
  if (!shader.loadNormals(NEIGHBORS))
  {
    shader.calcNormals(NEIGHBORS);
    timer.printTotalTime();
  }
  else
    cout << "Loaded normals from disc" << endl;
  // create textures without shadows
  cout << "Creating texture" << endl;
  timer = Timer();
  shader.createTextures(true, false);
  timer.printTotalTime();

  printSeparator('-');

  cout << "SHADING (" + save_dir + ") - NORMALS BY SAMPLING" << endl;
  if (!shader.loadNormals(SAMPLING))
  {
    timer = Timer();
    timer.printStartTime();
    shader.calcNormals(SAMPLING);
    timer.printTotalTime();
    // output of ratio and reset static timers
    TimerHandler::printRatio();
    TimerHandler::reset();
  }
  else
    cout << "Loaded normals from disc" << endl;
  // create textures without shadows
  cout << "Creating texture" << endl;
  timer = Timer();
  shader.createTextures(true, false);
  timer.printTotalTime();

  printSeparator('-');

  cout << "SHADING (" + save_dir + ") - NORMALS BY HYBRID" << endl;
  if (!shader.loadNormals(HYBRID))
  {
    timer = Timer();
    timer.printStartTime();
    shader.calcNormals(HYBRID);
    timer.printTotalTime();
    // output of ratio and reset static timers
    TimerHandler::printRatio();
    TimerHandler::reset();
  }
  else
    cout << "Loaded normals from disc" << endl;
  // create textures without shadows
  cout << "Creating texture" << endl;
  timer = Timer();
  shader.createTextures(true, false);
  timer.printTotalTime();

  printSeparator('-');

  cout << "SHADOWS (" + save_dir + ")" << endl;
  if (!shader.loadShadows())
  {
    timer = Timer();
    timer.printStartTime();
    shader.calcShadows();
    timer.printTotalTime();
    // output of ratio and reset static timers
    TimerHandler::printRatio();
    TimerHandler::reset();
  }
  else
    cout << "Loaded shadows from disc" << endl;
  // create textures without and with shading
  cout << "Creating texture (shadow only)" << endl;
  timer = Timer();
  shader.createTextures(false, true);
  timer.printTotalTime();

  printSeparator('-');

  cout << "SHADOW SHARPENING (" + save_dir + ")" << endl;
  if (do_shadow_sharpening)
  {
    timer = Timer();
    timer.printStartTime();
    shader.sharpenShadows();
    timer.printTotalTime();
    // output of ratio and reset static timers
    TimerHandler::printRatio();
    TimerHandler::reset();
  }
  else
    cout << "Skipped" << endl;

  // create textures without shading
  cout << "Creating texture (shadow only)" << endl;
  timer = Timer();
  shader.createTextures(false, true);
  timer.printTotalTime();
  // create final textures
  cout << "Creating texture (shadow and hybrid shading)" << endl;
  timer = Timer();
  shader.createTextures(true, true);
  timer.printTotalTime();

  printSeparator('=');
}

//--------------------------------------------------------------------------//
void executeDoubleGyreExperiments()
{
  // default setup
  SetupDoubleGyre3D setup{};
  // use scopes to automatically free memory as soon as the objects are no longer needed
  {
    // basic raytracing with smalest size
    auto dg_1 = basicRaytracing(setup.get_scene(), setup.create_cam(1), "dg/1");
    {
      // refining with doubled resolution
      auto dg_1_2 = refiningRaytracing(dg_1.get(), 2, "dg/1-2");
      // refining with doubled resolution of doubled resolution
      refiningRaytracing(dg_1_2.get(), 2, "dg/1-2-4");
    }
    // refining with quadrupled resolution
    refiningRaytracing(dg_1.get(), 4, "dg/1-4");
  }
  {
    // basic raytracing with doubles resolution
    auto dg_2 = basicRaytracing(setup.get_scene(), setup.create_cam(2), "dg/2");
    // refining with doubles resolution
    refiningRaytracing(dg_2.get(), 2, "dg/2-4");
  }
  {
    // basic raytracing with quadrupled resolution
    auto dg_4 = basicRaytracing(setup.get_scene(), setup.create_cam(4), "dg/4");
    // execute shading experiments
    shading(dg_4, "dg/4");
  }
}

//--------------------------------------------------------------------------//
void executeSquaredCylinderExperiments()
{
  // use scopes to automatically free memory as soon as the objects are no longer needed
  {
    // first try is with undersampling
    SetupSquaredCylinder setup{0.005, 0.2};
    basicRaytracing(setup.get_scene(), setup.create_cam(1), "sc/undersampled-1");
  }
  // default setup with better sampling
  SetupSquaredCylinder setup{};
  {
    // basic raytracing with smaller resolution
    auto sc_1 = basicRaytracing(setup.get_scene(), setup.create_cam(1), "sc/1");
    // refining with doubled resolution
    auto sc_1_2 = refiningRaytracing(sc_1.get(), 2, "sc/1-2");
  }
  {
    // basic raytracing with doubled resolution
    auto sc_2 = basicRaytracing(setup.get_scene(), setup.create_cam(2), "sc/2");
    // execute shading experiments
    shading(sc_2, "sc/2");
  }
}

//--------------------------------------------------------------------------//
int main(int /*argc*/, char ** /*argv[]*/)
{
  printSeparator('=');

  // set Globals::NEIGHBOR_DIFT0_PERLU and
  // Globals::NEIGHBOR_DIFTAU_PERLU to corresponding values

  executeDoubleGyreExperiments();
  // executeSquaredCylinderExperiments();
}
//--------------------------------------------------------------------------//