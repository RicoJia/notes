/**
 * controller.ts has endpoints 
 *  */ 
import { Controller, Get } from '@nestjs/common';
import { ApiResponse, ApiTags } from '@nestjs/swagger';

@ApiTags('app')
@Controller()
export class AppController {
  @Get('hello')
  // description goes into description of swagger
  @ApiResponse({ status: 200, description: 'Rico says: Return hello world.' })
  getHello(): string {
    return 'Hello World!';
  }
}