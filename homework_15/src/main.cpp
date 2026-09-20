#include "httplib.h"
#include "json.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

namespace fs = std::filesystem;
using json = nlohmann::json;

constexpr char SERVER_URL[] = "http://cppmiltech.com.ua";
constexpr char API_KEY[] = "dz12-vX7mK4qT9r2w";
constexpr char STUDENT_ID[] = "2117";
constexpr char POST_PATH[] = "/api/dz12/results";
constexpr int MAX_ATTEMPTS = 5;

struct ReportRow {
    std::string testId;
    std::string status;
    int attempts = 0;
};

std::string makeTestId(int number) {
    std::ostringstream output;
    output << 'T' << std::setw(2) << std::setfill('0') << number;
    return output.str();
}

bool readSimulation(const fs::path& path, json& simulation, std::string& error) {
    std::ifstream input(path);
    if (!input) {
        error = "cannot open file";
        return false;
    }

    try {
        input >> simulation;
    } catch (const json::exception& exception) {
        error = exception.what();
        return false;
    }

    if (!simulation.is_object()) {
        error = "simulation root must be a JSON object";
        return false;
    }

    return true;
}

bool isSuccessfulStatus(int status) {
    return status >= 200 && status < 300;
}

bool isRetryableError(httplib::Error error) {
    return error == httplib::Error::ConnectionTimeout
        || error == httplib::Error::Timeout
        || error == httplib::Error::Read;
}

void waitBeforeRetry(const std::string& testId,
                     const std::string& reason,
                     int attempt) {
    std::cout << testId << ": " << reason
              << ", attempt " << attempt << '/' << MAX_ATTEMPTS
              << ", retry after 1 second\n";
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

void printResponseBody(const std::string& testId,
                       const httplib::Response& response) {
    if (!response.body.empty()) {
        std::cerr << testId << ": server response: " << response.body << '\n';
    }
}

ReportRow uploadAndVerify(const std::string& testId,
                          const json& simulation) {
    ReportRow report{testId, "FAILED", 0};
    const json requestBody{
        {"studentId", STUDENT_ID},
        {"testId", testId},
        {"simulation", simulation}
    };
    const httplib::Headers headers{{"x-api-key", API_KEY}};

    httplib::Client client(SERVER_URL);
    client.set_connection_timeout(2, 0);
    client.set_read_timeout(2, 0);

    const std::string getPath =
        std::string("/api/dz12/results/") + testId + '/' + STUDENT_ID;

    for (int attempt = 1; attempt <= MAX_ATTEMPTS; ++attempt) {
        report.attempts = attempt;
        const auto postResult = client.Post(
            POST_PATH, headers, requestBody.dump(), "application/json");

        if (!postResult) {
            const auto error = postResult.error();
            const std::string reason = std::string("POST network error: ")
                + httplib::to_string(error);
            if (isRetryableError(error) && attempt < MAX_ATTEMPTS) {
                waitBeforeRetry(testId, reason, attempt);
                continue;
            }
            report.status = isRetryableError(error)
                ? "POST_TIMEOUT_AFTER_5" : "POST_NETWORK_ERROR";
            std::cerr << testId << ": " << reason << '\n';
            return report;
        }

        if (postResult->status == 503) {
            if (attempt < MAX_ATTEMPTS) {
                waitBeforeRetry(testId, "POST HTTP 503", attempt);
                continue;
            }
            report.status = "POST_HTTP_503_AFTER_5";
            return report;
        }

        if (postResult->status == 400 || postResult->status == 401) {
            report.status = "POST_HTTP_" + std::to_string(postResult->status);
            printResponseBody(testId, *postResult);
            return report;
        }

        if (!isSuccessfulStatus(postResult->status)) {
            report.status = "POST_HTTP_" + std::to_string(postResult->status);
            printResponseBody(testId, *postResult);
            return report;
        }

        const auto getResult = client.Get(getPath, headers);
        if (!getResult) {
            const auto error = getResult.error();
            const std::string reason = std::string("GET network error: ")
                + httplib::to_string(error);
            if (isRetryableError(error) && attempt < MAX_ATTEMPTS) {
                waitBeforeRetry(testId, reason, attempt);
                continue;
            }
            report.status = isRetryableError(error)
                ? "GET_TIMEOUT_AFTER_5" : "GET_NETWORK_ERROR";
            std::cerr << testId << ": " << reason << '\n';
            return report;
        }

        if (getResult->status == 503) {
            if (attempt < MAX_ATTEMPTS) {
                waitBeforeRetry(testId, "GET HTTP 503", attempt);
                continue;
            }
            report.status = "GET_HTTP_503_AFTER_5";
            return report;
        }

        if (getResult->status != 200) {
            report.status = "GET_HTTP_" + std::to_string(getResult->status);
            printResponseBody(testId, *getResult);
            return report;
        }

        try {
            const json verification = json::parse(getResult->body);
            report.status = verification.value("found", false)
                ? "VERIFIED" : "GET_FOUND_FALSE";
        } catch (const json::exception& exception) {
            report.status = "GET_INVALID_JSON";
            std::cerr << testId << ": invalid GET response: "
                      << exception.what() << '\n';
        }
        return report;
    }

    return report;
}

void printReport(const std::vector<ReportRow>& rows) {
    std::cout << "\nReport\n"
              << std::left << std::setw(8) << "Test"
              << std::setw(28) << "Status" << "Attempts\n";
    for (const auto& row : rows) {
        std::cout << std::left << std::setw(8) << row.testId
                  << std::setw(28) << row.status
                  << row.attempts << '\n';
    }
}

} // namespace

int main(int argc, char* argv[]) {
    if (argc > 2) {
        std::cerr << "Usage: " << argv[0] << " [results_directory]\n";
        return 1;
    }

    const fs::path resultsDirectory =
        argc == 2 ? fs::path(argv[1]) : fs::path("homework_15/results");
    std::vector<ReportRow> report;
    report.reserve(10);

    for (int number = 1; number <= 10; ++number) {
        const std::string testId = makeTestId(number);
        const fs::path simulationPath =
            resultsDirectory / testId / "simulation.json";

        if (!fs::exists(simulationPath)) {
            report.push_back({testId, "SKIPPED_FILE_MISSING", 0});
            continue;
        }

        json simulation;
        std::string error;
        if (!readSimulation(simulationPath, simulation, error)) {
            std::cerr << testId << ": " << simulationPath
                      << ": " << error << '\n';
            report.push_back({testId, "INVALID_LOCAL_JSON", 0});
            continue;
        }

        report.push_back(uploadAndVerify(testId, simulation));
    }

    printReport(report);
    return 0;
}
