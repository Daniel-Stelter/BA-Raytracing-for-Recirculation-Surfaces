#include "hyperpoint.hh"

using namespace std;

//--------------------------------------------------------------------------//
namespace RS
{
  //-----------------------------------------------------------------------------------------------//
  const Vec3r &
  HyperPoint::getPos(void) const
  {
    return m_pos;
  }

  //-----------------------------------------------------------------------------------------------//
  HyperPoint::HyperPoint(const Vec3r &pos, FlowSampler3D *p_flowSampler)
      : m_pos(pos), mp_flowSampler(p_flowSampler)
  {
    assert(p_flowSampler != nullptr);
  }

  //-----------------------------------------------------------------------------------------------//
  HyperPoint::HyperPoint(const HyperPoint &point)
  {
    mp_flowSampler = point.mp_flowSampler;
    m_flowMaps = map<pair<real, real>, shared_ptr<FlowMap3D>>(point.m_flowMaps);
    m_pos = point.m_pos;
  }

  //-----------------------------------------------------------------------------------------------//
  HyperPoint &
  HyperPoint::operator=(const HyperPoint &point)
  {
    mp_flowSampler = point.mp_flowSampler;
    m_flowMaps = map<pair<real, real>, shared_ptr<FlowMap3D>>(point.m_flowMaps);
    m_pos = point.m_pos;
    return *this;
  }

  //-----------------------------------------------------------------------------------------------//
  HyperPoint::~HyperPoint() {}

  //-----------------------------------------------------------------------------------------------//
  FlowSampler3D *
  HyperPoint::getFlowSampler(void) const
  {
    return mp_flowSampler;
  }

  //-----------------------------------------------------------------------------------------------//
  void
  HyperPoint::setFlowSampler(FlowSampler3D *p_flowSampler)
  {
    mp_flowSampler = p_flowSampler;
  }

  //-----------------------------------------------------------------------------------------------//
  /**Returns a flow map with the given t0 and tau. The flow map is integrated, if
it was not computed previously.*/
  const std::shared_ptr<FlowMap3D>
  HyperPoint::getFlowMap(const real &t0, const real &tau)
  {
    // check if the wanted flow map is already there
    pair<real, real> timePair = make_pair(t0, tau);
    if (m_flowMaps.find(timePair) != m_flowMaps.end())
      return m_flowMaps[timePair];

    // if it is not there, compute it
    shared_ptr<FlowMap3D> flowMap = make_shared<FlowMap3D>();
    mp_flowSampler->sampleFlow(flowMap.get(), m_pos, t0, tau);
    m_flowMaps[timePair] = flowMap;
    return flowMap;
  }

  //-----------------------------------------------------------------------------------------------//
  void
  HyperPoint::addFlowMap(const real &t0,
                         const real &tau,
                         shared_ptr<FlowMap3D> flow_map)
  {
    pair<real, real> timePair = make_pair(t0, tau);
    m_flowMaps[timePair] = flow_map;
  }
  //--------------------------------------------------------------------------//
}
//--------------------------------------------------------------------------//
