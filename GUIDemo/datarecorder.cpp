
#include "datarecorder.h"

master_db::master_db(const std::string &_DB_folder_path,
                     const std::string &_datetime)
    : dbpath(_DB_folder_path + "master.db") {
  sqlite::database db(dbpath);

  db << "CREATE TABLE IF NOT EXISTS info "
        "(ID INTEGER PRIMARY KEY, DATETIME TEXT NOT NULL);";
  db << "INSERT OR IGNORE INTO info (ID, DATETIME) VALUES(1, " + _datetime +
            ");";
}

marineradar_db::marineradar_db(const std::string &_DB_folder_path,
                               const std::string &_datetime)
    : master_db(_DB_folder_path, _datetime),
      dbpath(_DB_folder_path + "marineradar.db"),
      insert_string_spoke(""),
      db(dbpath) {}

void marineradar_db::create_table() {
  std::vector<std::pair<std::string, std::string>> db_config_spoke = {
      {"azimuth_deg", "DOUBLE"},
      {"sample_range", "DOUBLE"},
      {"SpokeData", "BLOB"}};

  std::string str =
      "CREATE TABLE radar"
      "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
      " DATETIME    TEXT       NOT NULL";
  insert_string_spoke = "(DATETIME";

  for (auto const &[name, type] : db_config_spoke) {
    str += ", " + name + " " + type;
    insert_string_spoke += ", " + name;
  }
  str += ");";
  insert_string_spoke += ") ";
  db << str;

  std::vector<std::pair<std::string, std::string>> db_config_motion = {
      {"state_x", "DOUBLE"}, {"state_y", "DOUBLE"}, {"state_theta", "DOUBLE"},
      {"state_u", "DOUBLE"}, {"state_v", "DOUBLE"}, {"state_r", "DOUBLE"}};
  str =
      "CREATE TABLE motion"
      "(ID          INTEGER PRIMARY KEY AUTOINCREMENT,"
      " DATETIME    TEXT       NOT NULL";
  insert_string_motion = "(DATETIME";

  for (auto const &[name, type] : db_config_motion) {
    str += ", " + name + " " + type;
    insert_string_motion += ", " + name;
  }
  str += ");";
  insert_string_motion += ") ";

  db << str;

  // commits can be orders of magnitude faster with synchronous OFF
  db << "PRAGMA synchronous = OFF";

}  // create_table

void marineradar_db::update_spoke_table(const marineradar_db_data &update_data,
                                        const std::string &_datetime) {
  std::string str = "INSERT INTO radar";
  str += insert_string_spoke;
  str += "VALUES(";
  str += _datetime;
  str += ",? ,? ,? )";

  db << str << update_data.azimuth_deg << update_data.sample_range
     << update_data.spokedata;

}  // update_spoke_table

void marineradar_db::update_motion_table(const est_state_db_data &update_data,
                                         const std::string &_datetime) {
  std::string str = "INSERT INTO motion";
  str += insert_string_motion;
  str += "VALUES(";
  str += _datetime;
  str += ", ";
  str += master_db::to_string_with_precision<double>(update_data.state_x, 3);
  str += ", ";
  str += master_db::to_string_with_precision<double>(update_data.state_y, 3);
  str += ", ";
  str += std::to_string(update_data.state_theta);
  str += ", ";
  str += master_db::to_string_with_precision<double>(update_data.state_u, 3);
  str += ", ";
  str += master_db::to_string_with_precision<double>(update_data.state_v, 3);
  str += ", ";
  str += std::to_string(update_data.state_r);
  str += ");";

  db << str;

}  // update_spoke_table
