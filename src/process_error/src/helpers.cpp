#include <iostream>
#include <string>
#include <sstream>

double getDoubleInput(const std::string& question, const double def) {
   bool retry = true;
   double result = 0;

   std::cout << question << "? (double value, default: " << def << "): ";

   while (retry) {
      retry = false;
      
      std::string ans;
      std::getline(std::cin, ans);

      if (ans.empty()) {
         result = def;
         std::cout << "Using default value: " << def << std::endl;
      }

      std::stringstream ss(ans);
      retry = (ss >> result).fail() || std::cin.fail() || !ss.eof() || ans.empty();

      if (retry) {
         std::cout << "Please enter a valid number!" << std::endl;
         retry = true;
      }
   }

   return result;
}

bool getBoolInput(const std::string& question) {
   bool retry = true;
   bool result = false;

   std::cout << question << "? (y/n): ";

   while (retry) {
      retry = true;

      std::string ans;
      std::cin >> ans;

      /*if (std::cin.fail() || !std::cin.eof()) {
         std::cout << "Please enter y/n!" << std::endl;
      }*/
      if (ans == "y" || ans == "Y" || ans == "yes" || ans == "1") {
         result = true;
         retry = false;
      }
      else if (ans == "n" || ans == "N" || ans == "no" || ans == "0") {
         result = false;
         retry = false;
      }

      std::cin.clear();
      std::cin.ignore();
   }

   return result;
}
