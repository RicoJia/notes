"use strict";
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.FakeCarModuleModule = void 0;
const common_1 = require("@nestjs/common");
const fake_car_module_controller_1 = require("./fake_car_module.controller");
const fake_car_module_service_1 = require("./fake_car_module.service");
let FakeCarModuleModule = exports.FakeCarModuleModule = class FakeCarModuleModule {
};
exports.FakeCarModuleModule = FakeCarModuleModule = __decorate([
    (0, common_1.Module)({
        controllers: [fake_car_module_controller_1.FakeCarModuleController],
        // Important: add service as a provider for nest to resolve dependencies
        providers: [fake_car_module_service_1.FakeCarService],
    })
], FakeCarModuleModule);
