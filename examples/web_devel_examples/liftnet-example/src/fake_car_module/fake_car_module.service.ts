import { Injectable } from "@nestjs/common";
import { FakeCar } from "./entities/fake_car.entity";

// injectable means class can be managed by nest IoC container
@Injectable()
export class FakeCarService{
    private readonly fakeCars: FakeCar[] = [];
    create(car: FakeCar){
        this.fakeCars.push(car);        
    }

    getAll(){
        return this.fakeCars;
    }
    getById(id: number){
        return this.fakeCars[id]; 
    }
    delete(id: number){
        this.fakeCars.splice(id, 1); 
    }
}