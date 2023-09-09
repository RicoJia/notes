"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.data = void 0;
let CC_DATA = {
    "floorReadingSlave": 2,
    "ipAddress": '167.112.154.79',
    "port": 502,
    "floorPositionRegister": 0,
    "statusRegister": 0,
    "statusRegisterSize": 16,
    "statusRegisterSegmentNum": 3,
    "controlRegister": 2001,
    "controlRegisterSize": 16,
    "mode": 'active',
    "car_num": 14
};
;
exports.data = {
    // TODO: can we initialize the list here?
    report: [
        {
            id: '1',
            amount: 100,
            type: "INCOME" /* DummyReportType.INCOME */,
            updated_at: new Date()
        },
        {
            id: '2',
            amount: 200,
            type: "EXPENSE" /* DummyReportType.EXPENSE */,
            updated_at: new Date()
        }
    ]
};
// data.report.push({
// });
