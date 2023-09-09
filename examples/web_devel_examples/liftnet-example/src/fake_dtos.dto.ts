import { ApiProperty } from '@nestjs/swagger';
export class ReportDTO{
    // RJ: setting example values here
    @ApiProperty({ description: 'Unique identifier for the item', example: "1" })
    id: string;
    @ApiProperty({ description: 'Amount of money', example: 67 })
    amount: number;
}