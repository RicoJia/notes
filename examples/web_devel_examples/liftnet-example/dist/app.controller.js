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
exports.AppController = void 0;
/**
 * controller.ts has endpoints
 *  */
const common_1 = require("@nestjs/common");
const swagger_1 = require("@nestjs/swagger");
const fake_dtos_dto_1 = require("./fake_dtos.dto");
const app_service_1 = require("./app.service");
let AppController = exports.AppController = class AppController {
    constructor(appService) {
        this.appService = appService;
    }
    getAllCarStatus() {
        return {};
    }
    getSingleCarStatus() {
        return {};
    }
    callCar() {
    }
    // dummy report.  ========================================================
    // NOTE: all Params must be in the route
    getDummyReport(type, id) {
        // this is to show path parameter validtion
        console.log('id', typeof (id));
        const returnType = type === 'INCOME' ? "INCOME" /* DummyReportType.INCOME */ : "EXPENSE" /* DummyReportType.EXPENSE */;
        return this.appService.getDummyReport(returnType, String(id));
    }
    // post with JSON input (DTO)
    createDummyReport(reportDTO, type) {
        const typeInDTO = type === 'INCOME' ? "INCOME" /* DummyReportType.INCOME */ : "EXPENSE" /* DummyReportType.EXPENSE */;
        return this.appService.createDummyReport(reportDTO, typeInDTO);
    }
    update_dummy_report(reportDTO) {
        return this.appService.updateDummyReport(reportDTO);
    }
    delete_dummy_report(reportId) {
        return this.appService.deleteDummyReport(reportId);
    }
};
__decorate([
    (0, common_1.Get)('status/all')
    // description goes into description of swagger
    ,
    (0, swagger_1.ApiResponse)({ status: 200, description: 'Rico says: Return hello world.' }),
    __metadata("design:type", Function),
    __metadata("design:paramtypes", []),
    __metadata("design:returntype", void 0)
], AppController.prototype, "getAllCarStatus", null);
__decorate([
    (0, common_1.Get)('status/:id'),
    __metadata("design:type", Function),
    __metadata("design:paramtypes", []),
    __metadata("design:returntype", void 0)
], AppController.prototype, "getSingleCarStatus", null);
__decorate([
    (0, common_1.Post)('call/:id'),
    __metadata("design:type", Function),
    __metadata("design:paramtypes", []),
    __metadata("design:returntype", void 0)
], AppController.prototype, "callCar", null);
__decorate([
    (0, common_1.Get)('dummy-report/:type/:id'),
    __param(0, (0, common_1.Param)('type')),
    __param(1, (0, common_1.Param)('id', common_1.ParseIntPipe)),
    __metadata("design:type", Function),
    __metadata("design:paramtypes", [String, Number]),
    __metadata("design:returntype", void 0)
], AppController.prototype, "getDummyReport", null);
__decorate([
    (0, common_1.Post)("create-dummy-report"),
    __param(0, (0, common_1.Body)()),
    __param(1, (0, common_1.Param)('type')),
    __metadata("design:type", Function),
    __metadata("design:paramtypes", [fake_dtos_dto_1.ReportDTO, String]),
    __metadata("design:returntype", void 0)
], AppController.prototype, "createDummyReport", null);
__decorate([
    (0, common_1.Patch)("update-dummy-report"),
    __param(0, (0, common_1.Body)()),
    __metadata("design:type", Function),
    __metadata("design:paramtypes", [fake_dtos_dto_1.ReportDTO]),
    __metadata("design:returntype", void 0)
], AppController.prototype, "update_dummy_report", null);
__decorate([
    (0, common_1.HttpCode)(200),
    (0, common_1.Delete)("delete-dummy-report/:reportId"),
    __param(0, (0, common_1.Param)('reportId')),
    __metadata("design:type", Function),
    __metadata("design:paramtypes", [String]),
    __metadata("design:returntype", void 0)
], AppController.prototype, "delete_dummy_report", null);
exports.AppController = AppController = __decorate([
    (0, swagger_1.ApiTags)('app')
    // Can do @Controller('base_route')
    ,
    (0, common_1.Controller)()
    // class needed in app.module.ts
    ,
    __metadata("design:paramtypes", [app_service_1.AppService])
], AppController);
