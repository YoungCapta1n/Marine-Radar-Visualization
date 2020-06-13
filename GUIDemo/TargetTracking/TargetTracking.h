#ifndef TARGETTRACKING_H
#define TARGETTRACKING_H

#include <pyclustering/cluster/dbscan.hpp>
#include <pyclustering/utils/metric.hpp>

#include "RadarFiltering.h"
#include "TargetTrackingData.h"
#include "math_utils.h"
#include "timecounter.h"

#include <iostream>

template <int max_num_target = 20>
class TargetTracking : public RadarFiltering {
  using T_Vectord = std::array<double, max_num_target>;
  using T_Vectori = std::array<int, max_num_target>;

 public:
  TargetTracking(const AlarmZone &_AlarmZone,
                 const SpokeProcessdata &_SpokeProcessdata,
                 const TrackingTargetData &_TrackingTargetData,
                 const ClusteringData &_ClusteringData)
      : RadarFiltering(1, 1),
        Alarm_Zone(_AlarmZone),
        SpokeProcess_data(_SpokeProcessdata),
        TrackingTarget_Data(_TrackingTargetData),
        Clustering_data(_ClusteringData),
        TargetTracking_RTdata({
            SPOKESTATE::OUTSIDE_ALARM_ZONE,  // spoke_state
            {0},                             // targets_state
            {0},                             // targets_intention
            {0},                             // targets_x
            {0},                             // targets_y
            {0},                             // targets_square_radius
            {0},                             // targets_vx
            {0},                             // targets_vy
            {0},                             // targets_CPA_x
            {0},                             // targets_CPA_y
            {0}                              // targets_TCPA
        }) {}
  virtual ~TargetTracking() = default;

  // spoke data from marine radar
  // [_vessel_x_m, _vessel_y_m]: vessel position in the marine coordinate
  // _vessel_theta_rad: vessel orientation (rad)
  // [_vessel_speed_x, _vessel_speed_y]: vessel speed in the marine coordinate
  TargetTracking &AutoTracking(
      const uint8_t *_spoke_array, const std::size_t _array_size,
      const double _spoke_azimuth_deg, const double _samplerange_m,
      const double _vessel_x_m = 0.0, const double _vessel_y_m = 0.0,
      const double _vessel_theta_rad = 0.0, const double _vessel_speed_x = 0.0,
      const double _vessel_speed_y = 0.0) {
    static double previous_spoke_azimuth_rad = 0;
    static bool previous_IsInAlarmAzimuth = false;
    static timecounter _timer;

    double _spoke_azimuth_rad =
        Normalizeheadingangle(Degree2Rad(_spoke_azimuth_deg));

    // TODO: empirical value
    // setClusteringdata(_samplerange_m);

    if (std::abs(Normalizeheadingangle(_spoke_azimuth_rad -
                                       previous_spoke_azimuth_rad)) > 0.008) {
      bool current_IsInAlarmAzimuth = IsInAlarmAzimuth(_spoke_azimuth_rad);
      if (current_IsInAlarmAzimuth) {  // in the alarm azimuth
        std::vector<double> surroundings_onespoke_bearing_rad;
        std::vector<double> surroundings_onespoke_range_m;
        std::vector<double> surroundings_onespoke_x_m;
        std::vector<double> surroundings_onespoke_y_m;

        find_surroundings_spoke(
            _spoke_array, _array_size, _spoke_azimuth_rad, _samplerange_m,
            surroundings_onespoke_bearing_rad, surroundings_onespoke_range_m);
        convert_surroundings_to_marine(
            _vessel_x_m, _vessel_y_m, _vessel_theta_rad,
            surroundings_onespoke_bearing_rad, surroundings_onespoke_range_m,
            surroundings_onespoke_x_m, surroundings_onespoke_y_m);

        // append the surroundings in the alarm zone
        SpokeProcess_RTdata.surroundings_bearing_rad.insert(
            SpokeProcess_RTdata.surroundings_bearing_rad.end(),
            surroundings_onespoke_bearing_rad.begin(),
            surroundings_onespoke_bearing_rad.end());
        SpokeProcess_RTdata.surroundings_range_m.insert(
            SpokeProcess_RTdata.surroundings_range_m.end(),
            surroundings_onespoke_range_m.begin(),
            surroundings_onespoke_range_m.end());
        SpokeProcess_RTdata.surroundings_x_m.insert(
            SpokeProcess_RTdata.surroundings_x_m.end(),
            surroundings_onespoke_x_m.begin(), surroundings_onespoke_x_m.end());
        SpokeProcess_RTdata.surroundings_y_m.insert(
            SpokeProcess_RTdata.surroundings_y_m.end(),
            surroundings_onespoke_y_m.begin(), surroundings_onespoke_y_m.end());

        // check the spoke azimuth to determine spoke state
        if (previous_IsInAlarmAzimuth)
          TargetTracking_RTdata.spoke_state = SPOKESTATE::IN_ALARM_ZONE;
        else {
          TargetTracking_RTdata.spoke_state = SPOKESTATE::ENTER_ALARM_ZONE;
          // _timer.timeelapsed();
        }

      } else {                            // outside the alarm azimuth
        if (previous_IsInAlarmAzimuth) {  // leaving the alarm azimuth

          long int et_ms = _timer.timeelapsed();
          double sample_time = 0.001 * et_ms;

          sample_time = 2.5;

          // start to cluster and miniball
          ClusteringAndMiniBall(SpokeProcess_RTdata.surroundings_x_m,
                                SpokeProcess_RTdata.surroundings_y_m,
                                TargetDetection_RTdata.target_x,
                                TargetDetection_RTdata.target_y,
                                TargetDetection_RTdata.target_square_radius);

          RemoveImpossibleRadius(TargetDetection_RTdata);

          TargetTracking_RTdata = PredictMotion(
              TargetDetection_RTdata.target_x, TargetDetection_RTdata.target_y,
              TargetDetection_RTdata.target_square_radius, sample_time,
              TargetTracking_RTdata);

          SituationAwareness(_vessel_speed_x, _vessel_speed_y, _vessel_speed_x,
                             _vessel_speed_y, TargetTracking_RTdata);

          RemoveDuplicateTargets(TargetTracking_RTdata);

          TargetTracking_RTdata.spoke_state = SPOKESTATE::LEAVE_ALARM_ZONE;

        } else {
          SpokeProcess_RTdata.surroundings_bearing_rad.clear();
          SpokeProcess_RTdata.surroundings_range_m.clear();
          SpokeProcess_RTdata.surroundings_x_m.clear();
          SpokeProcess_RTdata.surroundings_y_m.clear();

          TargetTracking_RTdata.spoke_state = SPOKESTATE::OUTSIDE_ALARM_ZONE;
        }
      }
    }  // check if two azimuth is different

    previous_spoke_azimuth_rad = _spoke_azimuth_rad;
    previous_IsInAlarmAzimuth = IsInAlarmAzimuth(previous_spoke_azimuth_rad);

    return *this;
  }  // AutoTracking

  TargetTracking &TestClustering(const std::vector<double> &_surroundings_x,
                                 const std::vector<double> &_surroundings_y) {
    ClusteringAndMiniBall(_surroundings_x, _surroundings_y,
                          TargetDetection_RTdata.target_x,
                          TargetDetection_RTdata.target_y,
                          TargetDetection_RTdata.target_square_radius);

    RemoveImpossibleRadius(TargetDetection_RTdata);

    return *this;
  }  // TestClustering

  TargetTracking &TestSituationAware() {
    std::vector<double> _detected_target_x{0, 3, 3, 3, 8};
    std::vector<double> _detected_target_y{0, 1, 2, 5, 7};
    std::vector<double> _detected_target_radius{1, 2, 1, 1, 3};

    TargetTrackerRTdata<max_num_target> _previous_tracking_targets{
        SPOKESTATE::OUTSIDE_ALARM_ZONE,  // spoke_state
        {0},                             // targets_state
        {0},                             // targets_intention
        {0},                             // targets_x
        {0},                             // targets_y
        {0},                             // targets_square_radius
        {0},                             // targets_vx
        {0},                             // targets_vy
        {0},                             // targets_CPA_x
        {0},                             // targets_CPA_y
        {0}                              // targets_TCPA
    };

    _previous_tracking_targets.targets_state[1] = 2;
    _previous_tracking_targets.targets_x[1] = 1;
    _previous_tracking_targets.targets_y[1] = 1;
    _previous_tracking_targets.targets_square_radius[1] = 2;
    _previous_tracking_targets.targets_vx[1] = 1;
    _previous_tracking_targets.targets_vy[1] = 1;

    _previous_tracking_targets.targets_state[3] = 2;
    _previous_tracking_targets.targets_x[3] = 12;
    _previous_tracking_targets.targets_y[3] = 13;
    _previous_tracking_targets.targets_square_radius[3] = 2;
    _previous_tracking_targets.targets_vx[3] = -1;
    _previous_tracking_targets.targets_vy[3] = -2;

    _previous_tracking_targets.targets_state[4] = 2;
    _previous_tracking_targets.targets_x[4] = 0;
    _previous_tracking_targets.targets_y[4] = 0;
    _previous_tracking_targets.targets_square_radius[4] = 2;
    _previous_tracking_targets.targets_vx[4] = -0.5;
    _previous_tracking_targets.targets_vy[4] = -0.5;

    double _sample_time = 2.5;

    auto _new_tracking_targets = PredictMotion(
        _detected_target_x, _detected_target_y, _detected_target_radius,
        _sample_time, _previous_tracking_targets);

    SituationAwareness(10, 10, -3, -3, _new_tracking_targets);

    std::cout << "targets_state: ";
    for (const auto &s : _new_tracking_targets.targets_state)
      std::cout << s << ' ';
    std::cout << "\n targets_x: ";
    for (const auto &s : _new_tracking_targets.targets_x) std::cout << s << ' ';
    std::cout << "\n targets_y: ";
    for (const auto &s : _new_tracking_targets.targets_y) std::cout << s << ' ';
    std::cout << "\n targets_square_radius: ";
    for (const auto &s : _new_tracking_targets.targets_square_radius)
      std::cout << s << ' ';
    std::cout << "\n targets_vx: ";
    for (const auto &s : _new_tracking_targets.targets_vx)
      std::cout << s << ' ';
    std::cout << "\n targets_vy: ";
    for (const auto &s : _new_tracking_targets.targets_vy)
      std::cout << s << ' ';
    std::cout << "\n targets_CPA_x: ";
    for (const auto &s : _new_tracking_targets.targets_CPA_x)
      std::cout << s << ' ';
    std::cout << "\n targets_CPA_y: ";
    for (const auto &s : _new_tracking_targets.targets_CPA_y)
      std::cout << s << ' ';
    std::cout << "\n targets_TCPA: ";
    for (const auto &s : _new_tracking_targets.targets_TCPA)
      std::cout << s << ' ';

    return *this;
  }  // TestSituationAware

  TargetDetectionRTdata getTargetDetectionRTdata() const noexcept {
    return TargetDetection_RTdata;
  }  // getTargetDetectionRTdata

  SpokeProcessRTdata getSpokeProcessRTdata() const noexcept {
    return SpokeProcess_RTdata;
  }  // getSpokeProcessRTdata

  TargetTrackerRTdata<max_num_target> getTargetTrackerRTdata() const noexcept {
    return TargetTracking_RTdata;
  }  // getTargetTrackerRTdata

  double getsampletime() const noexcept {
    return SpokeProcess_data.sample_time;
  }  // getsampletime

  void setClusteringdata(double _p_radius,
                         std::size_t _p_minumum_neighbors = 2) {
    Clustering_data.p_radius = _p_radius;
    Clustering_data.p_minumum_neighbors = _p_minumum_neighbors;
  }  // setClusteringdata

 private:
  const AlarmZone Alarm_Zone;
  const SpokeProcessdata SpokeProcess_data;
  const TrackingTargetData TrackingTarget_Data;
  ClusteringData Clustering_data;

  TargetTrackerRTdata<max_num_target> TargetTracking_RTdata;
  SpokeProcessRTdata SpokeProcess_RTdata;
  TargetDetectionRTdata TargetDetection_RTdata;

  // calculate the CPA and TCPA of the targets
  // whose speed is larger than threhold.
  // If the target speed is smaller than threhold, the target is assumed to be
  // static, abd CPA is equal to target position
  void SituationAwareness(
      const double _vessel_x_m, const double _vessel_y_m,
      const double _vessel_vx, const double _vessel_vy,
      TargetTrackerRTdata<max_num_target> &_TargetTracking_RTdata) {
    for (int i = 0; i != max_num_target; ++i) {
      if (_TargetTracking_RTdata.targets_state[i] > 0) {  // IDLE is ignored
        double _square_speed =
            std::pow(_TargetTracking_RTdata.targets_vx[i], 2) +
            std::pow(_TargetTracking_RTdata.targets_vy[i], 2);
        if (_square_speed <=
            std::pow(TrackingTarget_Data.speed_threhold, 2)) {  // static target
          _TargetTracking_RTdata.targets_CPA_x[i] =
              _TargetTracking_RTdata.targets_x[i];
          _TargetTracking_RTdata.targets_CPA_y[i] =
              _TargetTracking_RTdata.targets_y[i];

          if (CheckSafeDistance(_vessel_x_m, _vessel_y_m,
                                _TargetTracking_RTdata.targets_x[i],
                                _TargetTracking_RTdata.targets_y[i]))
            _TargetTracking_RTdata.targets_intention[i] = 1;  // dangerous
          else
            _TargetTracking_RTdata.targets_intention[i] = 0;  // safe

        } else {  // moving target
          auto [_CPA_x, _CPA_y, _TCPA] =
              computeCPA(_vessel_x_m, _vessel_y_m, _vessel_vx, _vessel_vy,
                         _TargetTracking_RTdata.targets_x[i],
                         _TargetTracking_RTdata.targets_y[i],
                         _TargetTracking_RTdata.targets_vx[i],
                         _TargetTracking_RTdata.targets_vy[i]);

          std::cout << "CPA: " << _CPA_x << " " << _CPA_y << " " << _TCPA
                    << std::endl;

          _TargetTracking_RTdata.targets_CPA_x[i] = _CPA_x;
          _TargetTracking_RTdata.targets_CPA_y[i] = _CPA_y;
          _TargetTracking_RTdata.targets_TCPA[i] = _TCPA;

          if (_TCPA > 0) {  // collision may occur
            if (CheckSafeDistance(_vessel_x_m, _vessel_y_m, _CPA_x, _CPA_y))
              _TargetTracking_RTdata.targets_intention[i] = 1;  // dangerous
            else
              _TargetTracking_RTdata.targets_intention[i] = 0;  // safe
          }
        }
      }
    }  // end for loop

  }  // SituationAwareness

  // calculate the CPA and TCPA, regarding with boat A, in global coordinate
  // TCPA < 0 means there is no CPA
  std::tuple<double, double, double> computeCPA(
      const double boatA_position_x, const double boatA_position_y,
      const double boatA_speed_x, const double boatA_speed_y,
      const double boatB_position_x, const double boatB_position_y,
      const double boatB_speed_x, const double boatB_speed_y) {
    double initial_delta_x = boatA_position_x - boatB_position_x;
    double initial_delta_y = boatA_position_y - boatB_position_y;
    double delta_speed_x = boatA_speed_x - boatB_speed_x;
    double delta_speed_y = boatA_speed_y - boatB_speed_y;
    double initial_square_distance =
        initial_delta_x * initial_delta_x + initial_delta_y * initial_delta_y;
    double initial_square_rela_speed =
        delta_speed_x * delta_speed_x + delta_speed_y * delta_speed_y;

    /** empiricial value **/
    double max_search_time = std::sqrt(2 * initial_square_distance /
                                       (initial_square_rela_speed + 1));
    int max_search_index = 100;
    double search_time_step = max_search_time / max_search_index;
    // empiricial value

    double min_distance = initial_square_distance;
    double CPA_x = 0;
    double CPA_y = 0;
    double TCPA = 0;
    for (int i = 0; i != max_search_index; ++i) {
      double search_time = i * search_time_step;
      double distance = (search_time * delta_speed_x + initial_delta_x) *
                            (search_time * delta_speed_x + initial_delta_x) +
                        (search_time * delta_speed_y + initial_delta_y) *
                            (search_time * delta_speed_y + initial_delta_y);

      if (distance < min_distance) {
        min_distance = distance;
        TCPA = search_time;
        CPA_x = boatA_position_x + boatA_speed_x * search_time;
        CPA_y = boatA_position_y + boatA_speed_y * search_time;
      }
    }

    if (TCPA < 0.01) TCPA = -1;

    return {CPA_x, CPA_y, TCPA};
  }  // computeCPA

  // clustering for all points and find the miniball around each sets of points
  void ClusteringAndMiniBall(const std::vector<double> &_surroundings_x,
                             const std::vector<double> &_surroundings_y,
                             std::vector<double> &_target_x,
                             std::vector<double> &_target_y,
                             std::vector<double> &_target_radius) {
    // clustering for all points
    std::shared_ptr<pyclustering::dataset> p_data =
        std::make_shared<pyclustering::dataset>();
    std::shared_ptr<pyclustering::clst::dbscan_data> ptr_output_result =
        std::make_shared<pyclustering::clst::dbscan_data>();

    std::size_t num_surroundings = _surroundings_x.size();
    p_data->resize(num_surroundings);
    for (std::size_t i = 0; i != num_surroundings; ++i) {
      p_data->at(i) = {_surroundings_x[i], _surroundings_y[i]};
    }

    pyclustering::clst::dbscan clustering_solver(
        Clustering_data.p_radius, Clustering_data.p_minumum_neighbors);
    clustering_solver.process(*p_data, *ptr_output_result);
    const pyclustering::clst::cluster_sequence &actual_clusters =
        ptr_output_result->clusters();

    std::size_t num_actual_clusters = actual_clusters.size();

    // miniball for each cluster
    _target_x.resize(num_actual_clusters);
    _target_y.resize(num_actual_clusters);
    _target_radius.resize(num_actual_clusters);

    for (std::size_t index = 0; index != num_actual_clusters; ++index) {
      auto cluster = actual_clusters.at(index);

      int d = 2;                       // dimension
      std::size_t n = cluster.size();  // number of points

      double **ap = new double *[n];
      for (std::size_t j = 0; j < n; ++j) {
        double *p = new double[d];
        p[0] = _surroundings_x[cluster.at(j)];
        p[1] = _surroundings_y[cluster.at(j)];
        ap[j] = p;
      }
      // create an instance of Miniball
      Miniball::Miniball<
          Miniball::CoordAccessor<double *const *, const double *> >
          mb(d, ap, ap + n);

      // output results
      const double *center = mb.center();
      _target_x[index] = center[0];
      _target_y[index] = center[1];
      _target_radius[index] = mb.squared_radius();

      // clean up
      for (std::size_t j = 0; j < n; ++j) delete[] ap[j];
      delete[] ap;
    }

  }  // ClusteringAndMiniBall

  // motion prediction for radar-detected target (Staight line assumption)
  TargetTrackerRTdata<max_num_target> PredictMotion(
      const std::vector<double> &new_target_x,
      const std::vector<double> &new_target_y,
      const std::vector<double> &new_target_radius, const double sample_time,
      const TargetTrackerRTdata<max_num_target> &previous_tracking_target) {
    TargetTrackerRTdata<max_num_target> new_tracking_target =
        previous_tracking_target;

    auto match_targets_index =
        TargetIdentification(new_target_x, new_target_y, new_target_radius,
                             previous_tracking_target, sample_time);

    for (int i = 0; i != max_num_target; ++i) {
      int match_target_index = match_targets_index.at(i);
      if (match_target_index < 0) {  // unmatched
        switch (previous_tracking_target.targets_state[i]) {
          case 0:  // IDLE
          case 1:  // ACQUIRING
            new_tracking_target.targets_state[i] = 0;
            new_tracking_target.targets_x[i] = 0;
            new_tracking_target.targets_y[i] = 0;
            new_tracking_target.targets_vx[i] = 0;
            new_tracking_target.targets_vy[i] = 0;
            break;
          case 2:  // ACQUIRED
            new_tracking_target.targets_state[i] = 1;

            auto [new_x, new_vx, new_y, new_vy] = RadarFiltering::NormalFilter(
                previous_tracking_target.targets_x[i],
                previous_tracking_target.targets_vx[i],
                previous_tracking_target.targets_y[i],
                previous_tracking_target.targets_vy[i], sample_time);

            std::tie(new_tracking_target.targets_vx[i],
                     new_tracking_target.targets_vy[i]) =
                SpeedFloor(new_vx, new_vy);

            new_tracking_target.targets_x[i] = new_x;
            new_tracking_target.targets_y[i] = new_y;
            new_tracking_target.targets_square_radius[i] =
                previous_tracking_target.targets_square_radius[i];
            break;
        }
      } else {  // matched
        switch (previous_tracking_target.targets_state[i]) {
          case 0:  // IDLE
            new_tracking_target.targets_state[i] = 1;
            new_tracking_target.targets_x[i] =
                new_target_x.at(match_target_index);
            new_tracking_target.targets_vx[i] = 0;
            new_tracking_target.targets_y[i] =
                new_target_y.at(match_target_index);
            new_tracking_target.targets_vy[i] = 0;
            new_tracking_target.targets_square_radius[i] =
                new_target_radius.at(match_target_index);
            break;
          case 1:
          case 2:  //  ACQUIRING/ACQUIRED
            new_tracking_target.targets_state[i] = 2;

            auto [new_x, new_vx, new_y, new_vy] = RadarFiltering::NormalFilter(
                previous_tracking_target.targets_x[i],
                previous_tracking_target.targets_vx[i],
                new_target_x.at(match_target_index),
                previous_tracking_target.targets_y[i],
                previous_tracking_target.targets_vy[i],
                new_target_y.at(match_target_index), sample_time);

            std::tie(new_tracking_target.targets_vx[i],
                     new_tracking_target.targets_vy[i]) =
                SpeedFloor(new_vx, new_vy);

            new_tracking_target.targets_x[i] = new_x;
            new_tracking_target.targets_y[i] = new_y;
            new_tracking_target.targets_square_radius[i] =
                0.5 * (previous_tracking_target.targets_square_radius[i] +
                       new_target_radius.at(match_target_index));

            break;
        }
      }
    }

    return new_tracking_target;

  }  // PredictMotion

  // remove the small radius from the detected targets
  void RemoveImpossibleRadius(TargetDetectionRTdata &_TargetDetection_RTdata) {
    std::vector<double> new_target_x;
    std::vector<double> new_target_y;
    std::vector<double> new_target_square_radius;

    std::size_t num_detected_targets = _TargetDetection_RTdata.target_x.size();
    for (std::size_t i = 0; i != num_detected_targets; ++i) {
      double _target_square_radius =
          _TargetDetection_RTdata.target_square_radius[i];
      if ((TrackingTarget_Data.min_squared_radius <= _target_square_radius) &&
          (_target_square_radius <= TrackingTarget_Data.max_squared_radius)) {
        new_target_x.emplace_back(_TargetDetection_RTdata.target_x[i]);
        new_target_y.emplace_back(_TargetDetection_RTdata.target_y[i]);
        new_target_square_radius.emplace_back(_target_square_radius);
      }
    }

    _TargetDetection_RTdata.target_x = new_target_x;
    _TargetDetection_RTdata.target_y = new_target_y;
    _TargetDetection_RTdata.target_square_radius = new_target_square_radius;

  }  // RemoveImpossibleRadius

  // remove the duplicate tracking targets, depending on target distance
  void RemoveDuplicateTargets(
      TargetTrackerRTdata<max_num_target> &_tracking_target) {
    for (int i = 0; i < (max_num_target - 1); ++i) {
      if (_tracking_target.targets_state[i] > 0)
        for (int j = (i + 1); j < max_num_target; ++j) {
          if (_tracking_target.targets_state[j] > 0) {
            double _distance = std::pow(_tracking_target.targets_x[i] -
                                            _tracking_target.targets_x[j],
                                        2) +
                               std::pow(_tracking_target.targets_y[i] -
                                            _tracking_target.targets_y[j],
                                        2);
            if (_distance <= std::pow(Clustering_data.p_radius, 2)) {
              double speed_i = std::pow(_tracking_target.targets_vx[i], 2) +
                               std::pow(_tracking_target.targets_vy[i], 2);
              double speed_j = std::pow(_tracking_target.targets_vx[j], 2) +
                               std::pow(_tracking_target.targets_vy[j], 2);
              if (speed_i > speed_j) {
                _tracking_target.targets_state[j] = 0;
              } else {
                _tracking_target.targets_state[i] = 0;
                break;
              }
            }  // if two targets are too close
          }
        }  // end for loop
    }      // end for loop

  }  // RemoveDuplicateTargets

  // match the detected target with the tracking targets
  T_Vectori TargetIdentification(
      const std::vector<double> &detected_target_x,
      const std::vector<double> &detected_target_y,
      const std::vector<double> &detected_target_radius,
      const TargetTrackerRTdata<max_num_target> &previous_tracking_targets,
      const double sample_time) {
    // match_index: -1 means unmatched, >=0 means the index in detected targets
    T_Vectori match_targets_index;
    match_targets_index.fill(-1);
    double inverse_time = 1.0 / sample_time;

    int num_detected_targets = static_cast<int>(detected_target_x.size());
    std::vector<int> unmatch_detected_targets_index(num_detected_targets);
    for (int i = 0; i != num_detected_targets; ++i) {
      unmatch_detected_targets_index[i] = i;
    }
    // first loop to match the detected targets with tracking
    // targets(ACQUIRING/ACQUIRED)
    for (int j = 0; j != max_num_target; ++j) {
      if (previous_tracking_targets.targets_state[j] > 0) {
        int index_match_with_detection = TargetIdentification(
            detected_target_x, detected_target_y, detected_target_radius,
            previous_tracking_targets.targets_x[j],
            previous_tracking_targets.targets_y[j],
            previous_tracking_targets.targets_square_radius[j],
            previous_tracking_targets.targets_vx[j],
            previous_tracking_targets.targets_vy[j], inverse_time);

        if (index_match_with_detection < num_detected_targets) {
          match_targets_index[j] = index_match_with_detection;
          if (index_match_with_detection >= 0)
            unmatch_detected_targets_index.erase(
                std::remove(unmatch_detected_targets_index.begin(),
                            unmatch_detected_targets_index.end(),
                            index_match_with_detection),
                unmatch_detected_targets_index.end());
        }
      }
    }

    // std::cout << "match index:\n " << match_targets_index << std::endl;

    // std::cout << "unmatched: \n";
    // for (auto const &value : unmatch_detected_targets_index) {
    //   std::cout << " " << value;
    // }
    // std::cout << std::endl;

    //  assign the unmatched detected targets to IDLE tracking targets
    std::size_t it_unmatched = 0;
    for (int j = 0; j != max_num_target; ++j) {
      if ((previous_tracking_targets.targets_state[j] == 0) &&
          (it_unmatched < unmatch_detected_targets_index.size())) {
        // only the IDLE previous tracking targets can be setup new detection
        match_targets_index[j] = unmatch_detected_targets_index[it_unmatched];
        ++it_unmatched;
      }
    }

    return match_targets_index;

  }  // TargetIdentification

  // match the detected targets with tracking targets(except IDLE)
  int TargetIdentification(const std::vector<double> &detected_target_x,
                           const std::vector<double> &detected_target_y,
                           const std::vector<double> &detected_target_radius,
                           const double previous_target_x,
                           const double previous_target_y,
                           const double previous_target_square_radius,
                           const double previous_target_vx,
                           const double previous_target_vy,
                           const double inverse_time) {
    // match_index: -1 means unmatched, >=0 means the index in detected targets
    int match_index = -1;
    double min_loss = 1e4;

    std::size_t num_detected_targets = detected_target_x.size();
    double Vj0_x = previous_target_vx;
    double Vj0_y = previous_target_vy;
    double previous_speed_j0 = std::hypot(Vj0_x, Vj0_y);

    double Vt = TrackingTarget_Data.speed_threhold;
    if (previous_speed_j0 > Vt) {
      for (std::size_t i = 0; i != num_detected_targets; ++i) {
        double Vji_x =
            inverse_time * (detected_target_x[i] - previous_target_x);
        double Vji_y =
            inverse_time * (detected_target_y[i] - previous_target_y);
        double predicted_speed_ji = std::hypot(Vji_x, Vji_y);

        double delta_velocity = std::hypot(Vji_x - Vj0_x, Vji_y - Vj0_y);

        double aji = inverse_time * delta_velocity;

        double delta_yaw = std::abs(VectorAngle_2d(Vj0_x, Vj0_y, Vji_x, Vji_y));

        // remove the detected targets which is un-matched
        if (predicted_speed_ji > TrackingTarget_Data.max_speed) continue;
        if (aji > TrackingTarget_Data.max_acceleration) continue;
        if ((delta_yaw * inverse_time) >
            (0.00029 * TrackingTarget_Data.max_roti))
          continue;

        double radius_term = std::abs(
            detected_target_radius[i] / previous_target_square_radius - 1);

        double delta_speed_term = delta_velocity / previous_speed_j0;

        double delta_angle_term = delta_yaw / M_PI;

        // std::cout << "i: " << i << " radius_term: " << radius_term
        //           << " delta_speed_term: " << delta_speed_term
        //           << " delta_angle_term: " << delta_angle_term << std::endl;
        // compute the loss
        double loss = TrackingTarget_Data.K_radius * radius_term +
                      TrackingTarget_Data.K_delta_speed * delta_speed_term +
                      TrackingTarget_Data.K_delta_yaw * delta_angle_term;
        if (loss < min_loss) {
          min_loss = loss;
          match_index = i;
        }
      }
    } else {  // previous speed is small, and the delta angle is ignored
      for (std::size_t i = 0; i != num_detected_targets; ++i) {
        double Vji_x =
            inverse_time * (detected_target_x[i] - previous_target_x);
        double Vji_y =
            inverse_time * (detected_target_y[i] - previous_target_y);

        double predicted_speed_ji = std::hypot(Vji_x, Vji_y);
        double delta_velocity = std::hypot(Vji_x - Vj0_x, Vji_y - Vj0_y);

        double aji = inverse_time * delta_velocity;

        // remove the detected targets which is un-matched
        if (predicted_speed_ji > TrackingTarget_Data.max_speed) continue;
        if (aji > TrackingTarget_Data.max_acceleration) continue;

        double radius_term = std::abs(
            detected_target_radius[i] / previous_target_square_radius - 1);
        double delta_speed_term = delta_velocity / Vt;

        // std::cout << "i: " << i << " radius_term: " << radius_term
        //           << " delta_speed_term: " << delta_speed_term << std::endl;

        // compute the loss
        double loss = TrackingTarget_Data.K_radius * radius_term +
                      TrackingTarget_Data.K_delta_speed * delta_speed_term;
        if (loss < min_loss) {
          min_loss = loss;
          match_index = i;
        }
      }
    }
    return match_index;
  }  // TargetIdentification

  // check the small speed which can be regarded as static
  std::tuple<double, double> SpeedFloor(const double _vx, const double _vy) {
    double new_vx = _vx;
    double new_vy = _vy;
    double _square_speed = _vx * _vx + _vy * _vy;
    if (_square_speed <= std::pow(TrackingTarget_Data.speed_threhold, 2)) {
      new_vx = 0;
      new_vy = 0;
    }
    return {new_vx, new_vy};
  }

  // check the safe distance, true: dangerous; false: safe
  bool CheckSafeDistance(double boat_x, double boat_y, double o_x, double o_y) {
    double _square_distance_V2T =
        std::pow(boat_x - o_x, 2) + std::pow(boat_y - o_y, 2);
    if (_square_distance_V2T <= std::pow(TrackingTarget_Data.safe_distance, 2))
      return true;
    return false;
  }  // CheckSafeDistance

  // check if spoke azimuth is the alarm zone, depending on azimuth
  bool IsInAlarmAzimuth(const double _current_spoke_azimuth_rad) {
    if (std::abs(Normalizeheadingangle(_current_spoke_azimuth_rad -
                                       Alarm_Zone.center_bearing_rad)) <=
        0.5 * Alarm_Zone.width_bearing_rad)
      return true;
    return false;
  }  // IsInAlarmAzimuth

  // convert the body-fixed coordinate to marine
  void convert_surroundings_to_marine(
      const double _vessel_x_m, const double _vessel_y_m,
      const double _vessel_theta_rad,
      const std::vector<double> &surroundings_bearing_rad,
      const std::vector<double> &surroundings_range_m,
      std::vector<double> &surroundings_x_m,
      std::vector<double> &surroundings_y_m) {
    double cvalue = std::cos(_vessel_theta_rad);
    double svalue = std::sin(_vessel_theta_rad);

    std::size_t num_surroundings = surroundings_bearing_rad.size();

    surroundings_x_m.resize(num_surroundings);
    surroundings_y_m.resize(num_surroundings);

    for (std::size_t i = 0; i != num_surroundings; ++i) {
      double bearing_rad = surroundings_bearing_rad[i];
      double range_m = surroundings_range_m[i];
      double cvalue_plus = std::cos(_vessel_theta_rad + bearing_rad);
      double svalue_plus = std::sin(_vessel_theta_rad + bearing_rad);

      double xs = cvalue * SpokeProcess_data.radar_x -
                  svalue * SpokeProcess_data.radar_y + range_m * cvalue_plus +
                  _vessel_x_m;

      double ys = svalue * SpokeProcess_data.radar_x +
                  cvalue * SpokeProcess_data.radar_y + range_m * svalue_plus +
                  _vessel_y_m;

      surroundings_x_m[i] = xs;
      surroundings_y_m[i] = ys;
    }
  }  // convert_surroundings_to_marine

  // find all surroundings on one spoke data, depending on the given threhold
  void find_surroundings_spoke(
      const uint8_t *_spoke_array, const std::size_t _array_size,
      const double _spoke_azimuth_rad, const double _samplerange_m,
      std::vector<double> &surroundings_InAlarm_bearing_rad,
      std::vector<double> &surroundings_InAlarm_range_m) {
    // find the index of all elements larger than threhold value
    auto surroundings_index = find_above_allelements_index<uint8_t>(
        _spoke_array, _array_size, Alarm_Zone.sensitivity_threhold);
    std::size_t num_surroundings = surroundings_index.size();

    // check if the surroundings are in the alarm zone
    surroundings_InAlarm_bearing_rad.clear();
    surroundings_InAlarm_range_m.clear();

    for (std::size_t i = 0; i != num_surroundings; ++i) {
      // TODO: test the empirical
      static double interception_empirical = 8.0;
      double s_range_m =
          interception_empirical + _samplerange_m * (surroundings_index[i] + 1);
      double s_bearing_rad = _spoke_azimuth_rad;

      if ((s_range_m <= Alarm_Zone.end_range_m) &&
          (Alarm_Zone.start_range_m <= s_range_m)) {
        surroundings_InAlarm_bearing_rad.emplace_back(s_bearing_rad);
        surroundings_InAlarm_range_m.emplace_back(s_range_m);
      }
    }

  }  // find_surroundings_spoke

  template <class T>
  std::vector<T> find_above_allelements(const T *arr, const int array_size,
                                        const T _threhold) {
    std::vector<T> target;
    std::copy_if(arr, arr + array_size, std::back_inserter(target),
                 [=](T element) { return element >= _threhold; });
    return target;
  }  // find_above_allelements

  template <class T>
  std::vector<std::size_t> find_above_allelements_index(
      const T *arr, const std::size_t array_size, const T _threhold) {
    std::vector<std::size_t> results;
    for (std::size_t i = 0; i != array_size; ++i) {
      if (arr[i] >= _threhold) results.emplace_back(i);
    }
    return results;
  }  // find_above_allelements_index

};  // end class TargetTracking

#endif  // TARGETTRACKING_H
