"use strict";
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.AppModule = void 0;
// A must for bootstrapping controller into the app module
const common_1 = require("@nestjs/common");
const app_controller_1 = require("./app.controller");
const app_service_1 = require("./app.service");
const fake_car_module_module_1 = require("./fake_car_module/fake_car_module.module");
// Configs
// RICO: envConfig: I still don't know what it is.
// import envConfig, { AppEnv, IConfiguration } from './config/env.config';
const config_1 = require("@nestjs/config");
let AppModule = exports.AppModule = class AppModule {
};
exports.AppModule = AppModule = __decorate([
    (0, common_1.Module)({
        imports: [
            fake_car_module_module_1.FakeCarModuleModule,
            config_1.ConfigModule.forRoot({
            //   isGlobal: true,
            //   cache: true
            // load: [envConfig],
            // envFilePath: process.env.APP_ENV === AppEnv.TEST ? DOT_ENV_TEST : undefined,
            })
        ],
        controllers: [app_controller_1.AppController],
        providers: [app_service_1.AppService],
    })
], AppModule);
