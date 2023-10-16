/**
 * controller.ts has endpoints 
 *  */ 
import { Controller, Get, Post, Param, Body, Patch, Delete, HttpCode, ParseIntPipe } from '@nestjs/common';
import { ApiResponse, ApiTags } from '@nestjs/swagger';
import { data, DummyReportType } from './data';
import { ReportDTO } from './fake_dtos.dto';
import { AppService } from './app.service'

@ApiTags('app')
// Can do @Controller('base_route')
@Controller()
// class needed in app.module.ts
export class AppController {
    // readonly: you can't change reference after construction except for object.assign
    // For deeper immutability, use Object.freeze()
    private readonly appService: AppService
    // Note: nest JS is fussy about access modifiers
    public constructor(
    ){}

  @Get('status/all')
  // description goes into description of swagger
  @ApiResponse({ status: 200, description: 'Rico says: Return hello world.' })
  getAllCarStatus(){
    return {};
  }

  @Get('status/:id')
  getSingleCarStatus(){
    return {};
  }

  @Post('call/:id')
  callCar(){

  }

  // dummy report.  ========================================================
  // NOTE: all Params must be in the route
  @Get('dummy-report/:type/:id')
  getDummyReport(
    @Param('type') type: string,
    @Param('id', ParseIntPipe) id: number
  ){
    // this is to show path parameter validtion
    console.log('id', typeof(id));
    const returnType = type === 'INCOME' ? DummyReportType.INCOME : DummyReportType.EXPENSE;
    return this.appService.getDummyReport(returnType, String(id));
  }

  // post with JSON input (DTO)
  @Post("create-dummy-report")
  createDummyReport(
    @Body() reportDTO : ReportDTO,
    @Param('type') type: string
  ){
    const typeInDTO = type === 'INCOME' ? DummyReportType.INCOME : DummyReportType.EXPENSE;
    return this.appService.createDummyReport(reportDTO, typeInDTO);
}

// @Body("DATA") is equivalent to (data?: string) => data ? req.body["DATA"] : req.body
@Patch("update-dummy-report")
update_dummy_report(
    @Body() reportDTO : ReportDTO,
){
    return this.appService.updateDummyReport(reportDTO);
}

@HttpCode(200)
@Delete("delete-dummy-report/:reportId")
delete_dummy_report(
    @Param('reportId') reportId: string
){
    return this.appService.deleteDummyReport(reportId);
}

// @ReqUser() user: User: extracts user after being authenticated

}
