#pragma once
#include <mysql.h>
#include <vector>
#include <string>
#include <iostream>

class ModelDatabase {
public:
	ModelDatabase(const char* host, unsigned int port, const char* user, const char* password) {
		conn = mysql_init(NULL);
		mysql_real_connect(conn, host, user, password, "modelsets", port, NULL, 0);
		if (conn == NULL) {
			std::cout << "connection failure" << std::endl;
		}
	}

	~ModelDatabase(){
		mysql_close(conn);
	}

	bool create_table(std::string model_name, std::vector<std::string> muscle_names) {
		if (muscle_names.size() <= 0) {
			return false;
		}

		std::string query = "create table " + model_name + " (angle DOUBLE PRIMARY KEY";
		for (int i = 0; i < muscle_names.size(); ++i) {
			query += (", " + muscle_names[i] + "_activ DOUBLE NOT NULL");
		}
		query += ");";

		if (mysql_query(conn, query.c_str()) > 0) {
			return false;
		}

		return true;
	}

	bool safe_data(std::string model_name, std::vector<double>& angle, std::vector<std::tuple<std::string, double>>& muscle_activations) {
		if (muscle_activations.size() <= 0) {
			return false;
		}

		//TODO uncomment and fix for vector insert
		//double db_angle = (double)((int)(angle * 100 + 0.5)) / 100;
		double db_angle = 0.0;

		std::string query = "INSERT INTO " + model_name + "(angle, ";

		for (int i = 0; i < muscle_activations.size(); ++i) {
			if (i == muscle_activations.size() - 1) {
				query += (std::get<0>(muscle_activations[i]) + "_activ");
				continue;
			}
			query += (std::get<0>(muscle_activations[i]) + "_activ, ");
		}
		query += (") VALUES (" + std::to_string(db_angle) + ", ");

		for (int i = 0; i < muscle_activations.size(); ++i) {
			if (i == muscle_activations.size() - 1) {
				query += std::to_string(std::get<1>(muscle_activations[i]));
				continue;
			}
			query += std::to_string(std::get<1>(muscle_activations[i])) + ", ";
		}

		query += ");";

		std::cout << query << std::endl;

		if (mysql_query(conn, query.c_str())) {
			return false;
		}

		return true;
	}

	std::vector<std::tuple<std::string, double>> get_activation(std::string model_name, double angle) {
		std::vector<std::tuple<std::string, double>> result_vector;

		double db_angle = (double)((int)(angle * 100 + 0.5)) / 100;

		std::string query = "SELECT * FROM " + model_name + " WHERE angle = " + std::to_string(db_angle) + ";";
		if (mysql_query(conn, query.c_str())) {
			return std::vector<std::tuple<std::string, double>>{};
		}

		MYSQL_RES* result = mysql_store_result(conn);
		int num_fields = mysql_num_fields(result);
		MYSQL_FIELD* field;
		MYSQL_ROW row;
		while (row = mysql_fetch_row(result)) {
			for (int i = 0; i < num_fields; ++i) {
				field = mysql_fetch_field(result);

				std::string field_name(field->name);
				std::string suffix("_activ");
				if (endswith(field_name, suffix)) {
					field_name.erase(field_name.find(suffix), suffix.length());
					
					std::tuple<std::string, double> t = std::make_tuple(field_name, std::stod(row[i]));
					result_vector.push_back(t);
				}
				else {
					std::tuple<std::string, double> t = std::make_tuple(field_name, std::stod(row[i]));
					result_vector.push_back(t);
				}
			}
		}

		return result_vector;
	}

	void close() {
		mysql_close(conn);
	}

	void connect(const char* host, unsigned int port, const char* user, const char* password) {
		conn = mysql_init(NULL);
		mysql_real_connect(conn, host, user, password, "modelsets", port, NULL, 0);
		if (conn == NULL) {
			std::cout << "connection failure" << std::endl;
		}
	}

	bool is_connected() {
		return conn != NULL;
	}

private:
	bool endswith(std::string& s1, std::string& s2) {
		if (s2.length() > s1.length()) {
			return false;
		}
		else {
			return 0 == s1.compare(s1.length() - s2.length(), s2.length(), s2);
		}
	}
	

private:
	MYSQL* conn;
};