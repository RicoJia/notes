import { Controller, Get, Param, Post, Body } from '@nestjs/common';
import { ApiTags } from "@nestjs/swagger";
import { FakeCarService } from './fake_car_module.service';
import {FakeCar} from './entities/fake_car.entity';

@ApiTags("fake_car_module")
@Controller('v1/cars')
export class FakeCarModuleController {
    /*
    * create a private readonly fakeCarService
    * In typescript, member variables are singleton?
    */
    constructor(private readonly fakeCarService: FakeCarService) {
    }

    @Get()
    getAllCars(){
        return this.fakeCarService.getAll();
    }

    // Defines the HTTP methods
    @Get(":id")
    getCarById(@Param('id') id: number){
        return this.fakeCarService.getById(id);
    }

    // On swagger, there's no function name. They are distinguished by 
    // 1. Submodules (query endpoint)
    // 2. Input types
    // 3. GET/POST, etc.
    @Post()
    // ? How do you create a fake car though curl call?
    createCar(@Body() car: FakeCar){
        this.fakeCarService.create(car); 
        // return json for API consistency, structured data, metadata
        // restful API favors json
        // when to return text: health check
        return {
            message: "car created",
            data: {
                id: car.id
            }
        };
    }
}
