// I think this is the convention we are following here.
// so swagger can see the definition of fakecar
import { ApiProperty } from '@nestjs/swagger';

export class FakeCar{
    @ApiProperty({ example: 123 })
    id: number;
    @ApiProperty({ example: 'Corolla' })
    model: string;
    @ApiProperty({ example: 2020 })
    year: number;
    @ApiProperty({ example: "Toyota"})
    manufacturer: string;
}
export enum CarStatus{
    PARKED = 'IDLE',
    FORWARD = 'FORWARD',
    REVERSE = 'REVERSE',
    NEUTRAL = 'NEUTRAL',
}