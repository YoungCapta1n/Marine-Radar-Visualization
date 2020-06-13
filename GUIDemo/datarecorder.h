#ifndef DATARECORDER_H
#define DATARECORDER_H

#include <sqlite_modern_cpp.h>
#include <sstream>

struct est_state_db_data {
  double local_time;
  double state_x;
  double state_y;
  double state_theta;
  double state_u;
  double state_v;
  double state_r;
};  // est_state_db_data

struct marineradar_db_data {
  double local_time;
  double azimuth_deg;
  double sample_range;
  std::vector<uint8_t> spokedata;
};  // marineradar_db_data

class master_db {
 public:
  explicit master_db(const std::string &_DB_folder_path,
                     const std::string &_datetime);
  virtual ~master_db() = default;

 protected:
  template <typename T>
  std::string to_string_with_precision(const T a_value, const int n = 6) {
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
  }

 private:
  std::string dbpath;

};  // end class master_db

class marineradar_db : public master_db {
 public:
  explicit marineradar_db(const std::string &_DB_folder_path,
                          const std::string &_datetime = "julianday('now')");
  ~marineradar_db() {}

  void create_table();

  void update_spoke_table(const marineradar_db_data &update_data,
                          const std::string &_datetime = "julianday('now')");

  void update_motion_table(const est_state_db_data &update_data,
                           const std::string &_datetime = "julianday('now')");

 private:
  std::string dbpath;
  std::string insert_string_spoke;
  std::string insert_string_motion;
  sqlite::database db;

};  // end class marineradar_db

#endif  // DATARECORDER_H
