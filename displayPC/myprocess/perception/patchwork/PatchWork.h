#ifndef PATCHWORK_H
#define PATCHWORK_H

#include "../perception/perceptioninterface.h"


struct PointXYZ
{
    float x;
    float y;
    float z;

    PointXYZ(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
};

struct RevertCandidate
{
  int concentric_idx;
  int sector_idx;
  double ground_flatness;
  double line_variable;
  Eigen::VectorXf pc_mean;
  std::vector<PointXYZ> regionwise_ground;

  RevertCandidate(int _c_idx, int _s_idx, double _flatness, double _line_var, Eigen::VectorXf _pc_mean, std::vector<PointXYZ> _ground)
   : concentric_idx(_c_idx), sector_idx(_s_idx), ground_flatness(_flatness), line_variable(_line_var), pc_mean(_pc_mean), regionwise_ground(_ground) {}
};

struct Params
{
  bool verbose;
  bool enable_RNR;
  bool enable_RVPF;
  bool enable_TGR;

  int num_iter;
  int num_lpr;
  int num_min_pts;
  int num_zones;
  int num_rings_of_interest;
  int noise_filter_channel_num;
  int pc_num_channel;

  double sensor_height;
  double th_seeds;
  double th_dist;
  double th_seeds_v;
  double th_dist_v;
  double max_range;
  double min_range;
  double uprightness_thr;
  double adaptive_seed_selection_margin;
  double intensity_thr;

  std::vector<int> num_sectors_each_zone;
  std::vector<int> num_rings_each_zone;

  int max_flatness_storage;
  int max_elevation_storage;

  std::vector<double> elevation_thr;
  std::vector<double> flatness_thr;


  Params()
  {
    verbose     = false;
    enable_RNR  = false;
    enable_RVPF = true;
    enable_TGR  = true;

    num_iter = 3;
    num_lpr = 20;
    num_min_pts = 10;
    num_zones = 4;
    num_rings_of_interest = 4;
    noise_filter_channel_num = 10;
    pc_num_channel = 1024;

    sensor_height = 0.000001;
    th_seeds = 0.125;
    th_dist = 0.125;
    th_seeds_v = 0.5;
    th_dist_v = 0.1;
    max_range = 35.0;
    min_range = 2.0;
    uprightness_thr =  0.307; // 0.707;
    adaptive_seed_selection_margin = -1.2;
    intensity_thr = 0.2;

    num_sectors_each_zone = {16, 32, 54, 32};
    num_rings_each_zone = {2, 4, 4, 4};
    max_flatness_storage = 1000;
    max_elevation_storage = 1000;
    elevation_thr = {0, 0, 0, 0};
    flatness_thr = {0, 0, 0, 0};
  }
};

// this is a class to realize specific function
class PatchWork
{
 public:
  PatchWork();
  ~PatchWork();

public:
  typedef std::vector<std::vector<PointXYZ>> Ring;
  typedef std::vector<Ring> Zone;

  void setParam(Params _params);

  void estimateGround(Eigen::MatrixX3f cloud_in);

  double getHeight() { return params_.sensor_height; }
  double getTimeTaken() { return time_taken_; }

  Eigen::MatrixX3f getGround() { return toEigenCloud(cloud_ground_); }
  Eigen::MatrixX3f getNonground() { return toEigenCloud(cloud_nonground_); }

  Eigen::MatrixX3f getCenters() { return toEigenCloud(centers_); }
  Eigen::MatrixX3f getNormals() { return toEigenCloud(normals_); }

 private:
   // Every private member variable is written with the undescore("_") in its end.

   Params params_;

   time_t timer_;
   long time_taken_;

   std::vector<double> update_flatness_[4];
   std::vector<double> update_elevation_[4];

   double d_; // when we estimate a plane, d is the fourth element in planar equation

   Eigen::VectorXf normal_;
   Eigen::VectorXf singular_values_; // Singular values sorted in decreasing order
   Eigen::Matrix3f cov_;
   Eigen::VectorXf pc_mean_; // center of a cloud

   std::vector<double> min_ranges_;
   std::vector<double> sector_sizes_;
   std::vector<double> ring_sizes_;

   std::vector<Zone> ConcentricZoneModel_;

   std::vector<PointXYZ> ground_pc_, non_ground_pc_;
   std::vector<PointXYZ> regionwise_ground_, regionwise_nonground_;

   std::vector<PointXYZ> cloud_ground_, cloud_nonground_;

   std::vector<PointXYZ> centers_, normals_;

   Eigen::MatrixX3f toEigenCloud(std::vector<PointXYZ> cloud);

   void addCloud(std::vector<PointXYZ> &cloud, std::vector<PointXYZ> &add);

   void flush_patches(std::vector<Zone> &czm); // reset Concentric Zone Model

   void pc2czm(const Eigen::MatrixX3f &src, std::vector<Zone> &czm); // devide cloud into rings and bins

   void reflected_noise_removal(Eigen::MatrixX3f &cloud_in, std::vector<PointXYZ> &nonground);

   void temporal_ground_revert(std::vector<PointXYZ> &cloud_ground, std::vector<PointXYZ> &cloud_nonground,
                               std::vector<double> ring_flatness, std::vector<RevertCandidate> candidates,
                               int concentric_idx);

   double calc_point_to_plane_d(PointXYZ p, Eigen::VectorXf normal, double d);
   void calc_mean_stdev(std::vector<double> vec, double &mean, double &stdev);

   void update_elevation_thr();
   void update_flatness_thr();

   double xy2theta(const double &x, const double &y);

   double xy2radius(const double &x, const double &y);

   // (in ground, in th_dist_v  = 0.1)
   void estimate_plane(const std::vector<PointXYZ> &ground, double th_dist);

   void extract_piecewiseground(const int zone_idx, const std::vector<PointXYZ> &src,
                                std::vector<PointXYZ> &dst, std::vector<PointXYZ> &non_ground_dst);
   // (in zone index, in cloud, out seed, in th_seeds_v = 0.5)
   void extract_initial_seeds(const int zone_idx, const std::vector<PointXYZ> &p_sorted,
                              std::vector<PointXYZ> &init_seeds); // init seed is a pointSet ;

   void extract_initial_seeds(const int zone_idx, const std::vector<PointXYZ> &p_sorted,
                              std::vector<PointXYZ> &init_seeds, double th_seed);

};

#endif // PATCHWORK_H
