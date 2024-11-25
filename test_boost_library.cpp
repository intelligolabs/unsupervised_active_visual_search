#include <iostream>
#include <boost/version.hpp>

int main() {
    // Retrieve Boost library version
    unsigned int major = BOOST_VERSION / 100000;
    unsigned int minor = (BOOST_VERSION / 100) % 1000;
    unsigned int patch = BOOST_VERSION % 100;

    // Print Boost version
    std::cout << "Boost version: " << major << "." << minor << "." << patch << std::endl;

    return 0;
}
