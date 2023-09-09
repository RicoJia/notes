interface SiteData{
    floorReadingSlave: number;
    ipAddress: string;
    port: number;
    floorPositionRegister: number;
    statusRegister: number;
    statusRegisterSize: number;
    statusRegisterSegmentNum: number;
    controlRegister: number;
    controlRegisterSize: number;
    mode: string;
    car_num: number;
} 

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
}

// ============================= Dummy Report =============================
export const enum DummyReportType{
    INCOME = 'INCOME',
    EXPENSE = `EXPENSE` 
};

interface Data {
    report: {
        id: string;
        amount: number;
        type: DummyReportType;
        // Rico: Below also works
        // type: 'INCOME' | 'EXPENSE';
        updated_at: Date;
    }[]
}

export const data: Data = {
    // TODO: can we initialize the list here?
    report: [
        {
            id: '1',
            amount: 100,
            type: DummyReportType.INCOME,
            updated_at: new Date()
        },
        {
            id: '2',
            amount: 200,
            type: DummyReportType.EXPENSE,
            updated_at: new Date()
        }
    ]
};

// data.report.push({
// });


