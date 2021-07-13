#include <iostream>
#include <memory>
#include "gtest/gtest.h"
#include "particle_filter.hpp"

using std::cout; using std::endl;

class ParticleFilterTest : public ::testing::Test{  //need :: for ::testing?
  protected:
    std::unique_ptr<Filter::Particle_Filter> pf_; 

    virtual void SetUp() override {
      pf_ = std::make_unique<Filter::Particle_Filter>(
        std::vector<std::pair<double, double>>{{1.0, 20.0}, {2.0, 40.0}}, 
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

    // attach observation callback
    try{
        pf_ -> register_observation_callback([](const std::vector<double>& observation){return 1.0;});
        pf_ -> run();
    }
    catch (const std::runtime_error& error){
        cout<<error.what()<<endl;
    };

    pf_ -> register_belief_callback([](const std::vector<double>& observation){}); 
    pf_ -> run(); 
    cout<<"Test Complete: "<<::testing::UnitTest::GetInstance()->current_test_info()->name()<<endl; 
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}

