import { Controller, Get } from '@nestjs/common';
import { ApiResponse, ApiTags } from '@nestjs/swagger';

@ApiTags('app')
@Controller()
export class AppController {
  @Get('hello')
  @ApiResponse({ status: 200, description: 'Return hello world.' })
  getHello(): string {
    return 'Hello World!';
  }
}