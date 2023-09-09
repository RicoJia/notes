"use strict";
/**
 * Rico: Business Logic should be in service
 */
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.AppService = void 0;
const common_1 = require("@nestjs/common");
const data_1 = require("./data");
// marking as injectable so dependency injection can figure it out
let AppService = exports.AppService = class AppService {
    getDummyReport(returnType, id) {
        return data_1.data.report.filter(r => r.type === returnType).find(r => r.id === id);
    }
    createDummyReport(reportDTO, type) {
        const newReport = {
            id: reportDTO.id,
            amount: reportDTO.amount,
            type,
            updated_at: new Date()
        };
        data_1.data.report.push(newReport);
        return newReport;
    }
    updateDummyReport(reportDTO) {
        const firstId = data_1.data.report.findIndex(r => r.id === reportDTO.id);
        if (firstId === -1) {
            throw new common_1.NotFoundException('report not found, request_id: ' + reportDTO.id);
        }
        else {
            data_1.data.report[firstId] = {
                ...data_1.data.report[firstId],
                ...reportDTO
            };
            return data_1.data.report[firstId];
        }
    }
    // return type could be undefined
    deleteDummyReport(reportId) {
        const firstId = data_1.data.report.findIndex(r => r.id === reportId);
        if (firstId === -1) {
            throw new common_1.NotFoundException('report not found, request_id: ' + reportId);
        }
        else {
            data_1.data.report.splice(firstId, 1);
        }
    }
};
exports.AppService = AppService = __decorate([
    (0, common_1.Injectable)()
], AppService);
