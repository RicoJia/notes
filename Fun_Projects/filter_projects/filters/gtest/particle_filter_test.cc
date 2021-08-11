#include <gtest/gtest_prod.h>
#include <iostream>
#include <memory>
#include "gtest/gtest.h"
#include "particle_filter.hpp"
#include <chrono>
#include <numeric>

using std::cout; using std::endl;

class ParticleFilterTest : public ::testing::Test{  //need :: for ::testing?
  protected:
    std::unique_ptr<Filter::Particle_Filter> pf_; 

    virtual void SetUp() override {
      pf_ = std::make_unique<Filter::Particle_Filter>(
        std::vector<double>{1.0, 20.0}, 
        10
      );
    }

    virtual void TearDown(){
    }
}; 

TEST_F(ParticleFilterTest, InitializationTest)
{
    // empty object test
    try{
        pf_ -> run(); 
    }
    catch (const std::runtime_error& error){
        cout<<error.what()<<endl;
    }; 

    try{
        pf_ -> register_control_callback([](std::vector<double>& state_vec){});
        pf_ -> run(); 
    }
    catch (const std::runtime_error& error){
        cout<<error.what()<<endl;
    }; 
    // attach observation callback
    try{
        pf_ -> register_observation_callback([](double& output_weight, const std::vector<double>& state_vec){output_weight = 1.0;});
        pf_ -> run();
    }
    catch (const std::runtime_error& error){
        cout<<error.what()<<endl;
    };

    pf_ -> run(); 
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}

