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
Object.defineProperty(exports, "__esModule", { value: true });
exports.CarStatus = exports.FakeCar = void 0;
// I think this is the convention we are following here.
// so swagger can see the definition of fakecar
const swagger_1 = require("@nestjs/swagger");
class FakeCar {
}
exports.FakeCar = FakeCar;
__decorate([
    (0, swagger_1.ApiProperty)({ example: 123 }),
    __metadata("design:type", Number)
], FakeCar.prototype, "id", void 0);
__decorate([
    (0, swagger_1.ApiProperty)({ example: 'Corolla' }),
    __metadata("design:type", String)
], FakeCar.prototype, "model", void 0);
__decorate([
    (0, swagger_1.ApiProperty)({ example: 2020 }),
    __metadata("design:type", Number)
], FakeCar.prototype, "year", void 0);
__decorate([
    (0, swagger_1.ApiProperty)({ example: "Toyota" }),
    __metadata("design:type", String)
], FakeCar.prototype, "manufacturer", void 0);
var CarStatus;
(function (CarStatus) {
    CarStatus["PARKED"] = "IDLE";
    CarStatus["FORWARD"] = "FORWARD";
    CarStatus["REVERSE"] = "REVERSE";
    CarStatus["NEUTRAL"] = "NEUTRAL";
})(CarStatus || (exports.CarStatus = CarStatus = {}));
