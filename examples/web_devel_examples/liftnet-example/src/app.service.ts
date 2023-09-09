/**
 * Rico: Business Logic should be in service
 */

import { Injectable, NotFoundException } from '@nestjs/common'
import { data, type DummyReportType } from './data'
import { type ReportDTO } from './fake_dtos.dto'

// marking as injectable so dependency injection can figure it out
@Injectable()
export class AppService {
  getDummyReport (returnType: DummyReportType, id: string): any {
    return data.report.filter(r => r.type === returnType).find(r => r.id === id)
  }

  createDummyReport (
    reportDTO: ReportDTO,
    type: DummyReportType
  ): any {
    const newReport = {
      id: reportDTO.id,
      amount: reportDTO.amount,
      type,
      updated_at: new Date()
    }

    data.report.push(newReport)
    return newReport
  }

  updateDummyReport (
    reportDTO: ReportDTO
  ): any {
    const firstId = data.report.findIndex(r => r.id === reportDTO.id)
    if (firstId === -1) {
      throw new NotFoundException('report not found, request_id: ' + reportDTO.id)
    } else {
      data.report[firstId] = {
        ...data.report[firstId],
        ...reportDTO
      }
      return data.report[firstId]
    }
  }

  // return type could be undefined
  deleteDummyReport (reportId: string): undefined {
    const firstId = data.report.findIndex(r => r.id === reportId)
    if (firstId === -1) {
      throw new NotFoundException('report not found, request_id: ' + reportId)
    } else {
      data.report.splice(firstId, 1)
    }
  }
}
