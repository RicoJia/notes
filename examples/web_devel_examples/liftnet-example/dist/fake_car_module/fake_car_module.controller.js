"use strict";
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};
var __param = (this && this.__param) || function (paramIndex, decorator) {
    return function (target, key) { decorator(target, key, paramIndex); }
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.FakeCarModuleController = void 0;
const common_1 = require("@nestjs/common");
const swagger_1 = require("@nestjs/swagger");
const fake_car_module_service_1 = require("./fake_car_module.service");
const fake_car_entity_1 = require("./entities/fake_car.entity");
let FakeCarModuleController = exports.FakeCarModuleController = class FakeCarModuleController {
    /*
    * create a private readonly fakeCarService
    * In typescript, member variables are singleton?
    */
    constructor(fakeCarService) {
        this.fakeCarService = fakeCarService;
    }
    getAllCars() {
        return this.fakeCarService.getAll();
    }
    // Defines the HTTP methods
    getCarById(id) {
        return this.fakeCarService.getById(id);
    }
    // On swagger, there's no function name. They are distinguished by 
    // 1. Submodules (query endpoint)
    // 2. Input types
    // 3. GET/POST, etc.
    // ? How do you create a fake car though curl call?
    createCar(car) {
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
};
__decorate([
    (0, common_1.Get)(),
    __metadata("design:type", Function),
    __metadata("design:paramtypes", []),
    __metadata("design:returntype", void 0)
], FakeCarModuleController.prototype, "getAllCars", null);
__decorate([
    (0, common_1.Get)(":id"),
    __param(0, (0, common_1.Param)('id')),
    __metadata("design:type", Function),
    __metadata("design:paramtypes", [Number]),
    __metadata("design:returntype", void 0)
], FakeCarModuleController.prototype, "getCarById", null);
__decorate([
    (0, common_1.Post)()
    // ? How do you create a fake car though curl call?
    ,
    __param(0, (0, common_1.Body)()),
    __metadata("design:type", Function),
    __metadata("design:paramtypes", [fake_car_entity_1.FakeCar]),
    __metadata("design:returntype", void 0)
], FakeCarModuleController.prototype, "createCar", null);
exports.FakeCarModuleController = FakeCarModuleController = __decorate([
    (0, swagger_1.ApiTags)("fake_car_module"),
    (0, common_1.Controller)('v1/cars'),
    __metadata("design:paramtypes", [fake_car_module_service_1.FakeCarService])
], FakeCarModuleController);
