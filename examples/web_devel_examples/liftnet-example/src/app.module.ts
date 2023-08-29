// A must for bootstrapping controller into the app module
import { Module } from '@nestjs/common';
import { AppController } from './app.controller';
import { FakeCarModuleModule } from './fake_car_module/fake_car_module.module';

@Module({
  imports: [FakeCarModuleModule],
  controllers: [AppController],
  providers: [],
})
export class AppModule {}
