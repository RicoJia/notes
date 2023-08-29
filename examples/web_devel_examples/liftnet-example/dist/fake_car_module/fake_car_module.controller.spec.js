"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
const testing_1 = require("@nestjs/testing");
const fake_car_module_controller_1 = require("./fake_car_module.controller");
describe('FakeCarModuleController', () => {
    let controller;
    beforeEach(async () => {
        const module = await testing_1.Test.createTestingModule({
            controllers: [fake_car_module_controller_1.FakeCarModuleController],
        }).compile();
        controller = module.get(fake_car_module_controller_1.FakeCarModuleController);
    });
    it('should be defined', () => {
        expect(controller).toBeDefined();
    });
});
