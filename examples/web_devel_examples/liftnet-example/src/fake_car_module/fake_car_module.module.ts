import { Module } from '@nestjs/common';
import { FakeCarModuleController } from './fake_car_module.controller';
import { FakeCarService } from './fake_car_module.service';

@Module({
  controllers: [FakeCarModuleController],
  // Important: add service as a provider for nest to resolve dependencies
  providers: [FakeCarService],
})
export class FakeCarModuleModule {}
